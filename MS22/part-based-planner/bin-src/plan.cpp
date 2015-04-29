#include <nuklei/KernelCollection.h>
#include <nuklei/ObservationIO.h>
#include <nuklei/ProgressIndicator.h>
#include <nuklei/PoseEstimator.h>
#include <nuklei/Stopwatch.h>
#include <tclap/CmdLine.h>
#include <pbp/UibkGraspTemplates.hpp>
#include <pbp/collission_checker.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <ros/console.h>
#include <log4cxx/logger.h>
#include <ros/message_operations.h>

#include <Eigen/Dense>


int main(int argc, char ** argv)
{
  try {
    
    using namespace nuklei;
    using namespace pbp;
    using namespace TCLAP;

    //ROS INITS:

    boost::uuids::basic_random_generator<boost::mt19937> gen;
    boost::uuids::uuid u = gen(); //generate uuid

    std::stringstream ss;
    ss << u;
    std::string random_string=ss.str();
    std::string s1 = "pbp_" + random_string.substr(1,5);

    ros::init(argc, argv, s1.c_str());
    //logger level
    log4cxx::LoggerPtr my_logger;
    my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);


    ros::AsyncSpinner spinner(1);
    spinner.start();
    robot_model_loader::RobotModelLoader robot_model_loader("/robot_description");
    robot_state::RobotModelPtr model = robot_model_loader.getModel();

    if(model == NULL){
        ROS_ERROR("No robot_description found - fixing error later!");

        exit(EXIT_FAILURE);
    }
    
    CmdLine cmd("");
    
    UnlabeledValueArg<std::string> objectFileArg
    ("object_evidence",
     "Object file.",
     true, "", "filename", cmd);
    
    UnlabeledValueArg<std::string> sceneFileArg
    ("scene_evidence",
     "Scene file.",
     true, "", "filename", cmd);
    
    ValueArg<std::string> roisFileArg
    ("", "rois",
     "Set of boxes, parametrized by their center, orientation and edge lenghts. "
     "No collisions between these boxes and other elements are allowed. "
     "Collisions that can be checked: collisions with tableplane, with the scene "
     "points, and with an optional obstacle file.",
     false, "", "filename", cmd);
    
    ValueArg<std::string> obstacleFileArg
    ("", "obst",
     "Set of points that cannot collide with the --rois arg.",
     false, "", "filename", cmd);

    ValueArg<double> backoffArg
    ("", "backoff_dist",
     "For 'reachable' cifs, check that ik also exist for a pose behind the "
     "gripping pose.",
     false, 0, "float", cmd);

    ValueArg<int> handApproachAxisArg
    ("", "hand_approach_axis",
     "Axis that points away from the hand's plam. 0 if x, 1 if y, 2 if z.",
     false, 0, "int", cmd);

    ValueArg<std::string> tableplaneFileArg
    ("", "table",
     "Table plane. Parametrized by a point and SO(3) ori whose z is normal to the plane.",
     false, "", "filename", cmd);

    ValueArg<std::string> meshFileArg
    ("", "mesh",
     "Mesh model of object evidence. Used only if --partial is enabled.",
     false, "", "filename", cmd);
    
    ValueArg<std::string> viewpointFileArg
    ("", "viewpoint",
     "File containing XYZ of the camera. Used only if --partial is enabled.",
     false, "", "filename", cmd);
    
    ValueArg<std::string> alignedObjectEvidenceFileArg
    ("", "aligned",
     "Transformed object evidence, matching object pose.",
     false, "", "filename", cmd);
    
    ValueArg<int> nArg
    ("n", "n_model_points",
     "Number of particle supporting the object model.",
     false, 0, "int", cmd);
    
    ValueArg<double> locHArg
    ("l", "loc_h",
     "Location kernel width.",
     false, 0, "float", cmd);
    
    ValueArg<double> oriHArg
    ("o", "ori_h",
     "Orientation kernel width (in radians).",
     false, 0.2, "float", cmd);
    
    ValueArg<int> nChainsArg
    ("c", "n_chains",
     "Number of MCMC chains.",
     false, 0, "int", cmd);
    
    ValueArg<std::string> bestTransfoArg
    ("", "best_transfo",
     "File to write the most likely transformation to.",
     false, "", "filename", cmd);

    ValueArg<std::string> resultFileArg
    ("", "log",
     "File to write the log to.",
     false, "", "filename", cmd);

    ValueArg<std::string> cifArg
    ("", "cif",
     "Select a custom integrand factor (a class that computes a factor that "
     "is multiplied into the shape matching function). Possible values are: "
     "torque, kth-reach, ur5-reach, uibk-inertia-reach.",
     false, "", "filename", cmd);

    ValueArg<double> meshVisibilityArg
    ("", "point_to_mesh_visibility_dist",
     "Sets the distance to the mesh at which a point is considered to be visible.",
     false, 4., "float", cmd);

    SwitchArg computeNormalsArg
    ("", "normals",
     "OBSOLETE ARGUMENT. NORMALS ARE ALWAYS COMPUTED. "
     "Compute a normal vector for all input points. "
     "Makes pose estimation more robust.", cmd);
    
    SwitchArg accurateScoreArg
    ("s", "accurate_score",
     "OBSOLETE ARGUMENT. ACCURATE SCORE IS ALWAYS COMPUTED. "
     "Recompute the matching score using all input points "
     "(instead of using N points as given by -n N).", cmd);
    
    SwitchArg timeArg
    ("", "time",
     "Print computation time.", cmd);
    
    SwitchArg partialviewArg
    ("", "partial",
     "Match only the visible side of the model to the object.", cmd);

    /////arguments for UIBK stuff:

    ValueArg<int> uibkArmArg
    ("", "arm-nr",
     "Which arm of uibk robot we are using, right=0=standard, left is=1",
     false, 0, "int", cmd);

    ValueArg<std::string> SDHJointsArg
    ("", "sdh",
     "sdh joints",
     false, "", "filename", cmd);

    ValueArg<std::string> BBoxClusterArg
    ("", "bboxC",
     ".roi file of part",
     false, "", "filename", cmd);

    ValueArg<std::string> BBoxTemplateArg
    ("", "bboxT",
     ".roi file template",
     false, "", "filename", cmd);


    
    cmd.parse( argc, argv );
    Stopwatch sw("");
    if (!timeArg.getValue())
      sw.setOutputType(Stopwatch::QUIET);
    
    // ------------- //
    // Read-in data: //
    // ------------- //
    
    KernelCollection obstacles;
    if (obstacleFileArg.getValue() != "")
      readObservations(obstacleFileArg.getValue(), obstacles);
    obstacles.computeKernelStatistics();
    
    KernelCollection roiPoses;
    std::vector<Vector3> roiWidths;
    
    if (roisFileArg.getValue() != "")
    {
      std::ifstream ifs(roisFileArg.getValue().c_str());
      std::string line;
      while (std::getline(ifs, line))
      {
        double tx, ty, tz, qw, qx, qy, qz, a, b, c;
        std::istringstream iss(line);
        if (!(iss >> tx >> ty >> tz >> qw >> qx >> qy >> qz >> a >> b >> c))
          break;
        
        kernel::se3 k;
        k.loc_ = Vector3(tx, ty, tz);
        k.ori_ = la::normalized(Quaternion(qw, qx, qy, qz));
        Vector3 v(a, b, c);
        roiPoses.add(k);
        roiWidths.push_back(v);
      }
    }

    //------------------//
    //Read-in UIBK data://
    //------------------//


    std::vector<double> handJoints; //maybe best to load hand joints of pregrasp
    bbox_eigen bbox_template, bbox_cluster; //load from somwhere
    //malloc(sizeof(*one))

    //loading bbounding boxes of templates and part
    if(BBoxClusterArg.getValue()!= "" && BBoxTemplateArg.getValue()!= ""){

        std::ifstream ifs(BBoxClusterArg.getValue().c_str());
        std::string line;
        while (std::getline(ifs, line))
        {
          float tx, ty, tz, qw, qx, qy, qz, a, b, c;
          std::istringstream iss(line);
          if (!(iss >> tx >> ty >> tz >> qw >> qx >> qy >> qz >> a >> b >> c))
            break;

          bbox_cluster.trans << tx, ty, tz;
          bbox_cluster.quat = Eigen::Quaternionf(qw,qx,qy,qz);
          bbox_cluster.scales << a,b,c;
        }

        std::ifstream ifs2(BBoxTemplateArg.getValue().c_str());
        std::string line2;
        while (std::getline(ifs2, line2))
        {
          float tx, ty, tz, qw, qx, qy, qz, a, b, c;
          std::istringstream iss2(line2);
          if (!(iss2 >> tx >> ty >> tz >> qw >> qx >> qy >> qz >> a >> b >> c))
            break;

          bbox_template.trans << tx, ty, tz;
          bbox_template.quat = Eigen::Quaternionf(qw,qx,qy,qz);
          bbox_template.scales << a,b,c;
        }

    }

    //------------------//
    
    
    boost::shared_ptr<CustomIntegrandFactor> cif;
    
    PoseEstimator pe(locHArg.getValue(),
                     oriHArg.getValue(),
                     nChainsArg.getValue(),
                     nArg.getValue(),
                     cif,
                     partialviewArg.getValue(),
                     true);
    pe.setMeshToVisibilityTol(meshVisibilityArg.getValue());
    
    pe.load(objectFileArg.getValue(),
            sceneFileArg.getValue(),
            meshFileArg.getValue(),
            viewpointFileArg.getValue(),
            true,
            true);
    
    if (cifArg.getValue() == "none" || cifArg.getValue() == "")
      ; // do nothing  
    else if (cifArg.getValue()=="uibk-grasp-templates")
        cif.reset(new UibkGraspTemplates(pe.getObjectModel(),uibkArmArg.getValue(), model));
    else NUKLEI_THROW("Unknown CIF `" << cifArg.getValue() << "'.");

    if (cifArg.getValue() == "uibk-grasp-templates")
    {

      UibkGraspTemplates* uibkGraspTemplates = dynamic_cast<UibkGraspTemplates*>(cif.get());

      uibkGraspTemplates->setRois(roiPoses, roiWidths);
      uibkGraspTemplates->setObstacles(obstacles);
      uibkGraspTemplates->setHandApproachAxis(handApproachAxisArg.getValue());
      uibkGraspTemplates->setBackoffDist(backoffArg.getValue());
      //new stuff
      uibkGraspTemplates->setBBoxes(bbox_cluster, bbox_template);

      if(SDHJointsArg.getValue() != "" && BBoxClusterArg.getValue() !="" && BBoxTemplateArg.getValue() != "" ){
       //All these guys have to be loaded!!

          uibkGraspTemplates->setJointsForSDH(handJoints);
          uibkGraspTemplates->setBBoxes(bbox_cluster, bbox_template);

      }
      else{
          ROS_ERROR("no hand joint arguments supplied");
      }

      if (tableplaneFileArg.getValue() != "")
      {
        kernel::se3 table(*readSingleObservation(tableplaneFileArg.getValue()));
        uibkGraspTemplates->setTablePlane(table);
      }
    }
    
    pe.setCustomIntegrandFactor(cif);
    sw.lap("data read");
    
    // ------------------------------- //
    // Prepare density for evaluation: //
    // ------------------------------- //
    
    
    kernel::se3 t = pe.modelToSceneTransformation();
    
    sw.lap("alignment");
    
    if (!resultFileArg.getValue().empty())
    {
      KernelCollection objectEvidence;
      readObservations(objectFileArg.getValue(), objectEvidence);
      
      std::ofstream ofs(resultFileArg.getValue().c_str());
      ofs.precision(PRECISION);
      ofs << "[pbp]";
      ofs << "\nscore: " << t.getWeight();
      ofs << "\nobject_model_n_points: " << objectEvidence.size();
      ofs << "\npose: " << stringify(t.loc_, PRECISION) + " " + stringify(t.ori_, PRECISION);
      ofs << std::endl;
    }
    
    if (!bestTransfoArg.getValue().empty())
    {
      writeSingleObservation(bestTransfoArg.getValue(), t);
    }
    
    if (!alignedObjectEvidenceFileArg.getValue().empty())
    {
      pe.writeAlignedModel(alignedObjectEvidenceFileArg.getValue(), t);
    }
    
    sw.lap("output");
    
    return 0;
  }
  catch (std::exception &e) {
    std::cerr << "Exception caught: ";
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (...) {
    std::cerr << "Caught unknown exception." << std::endl;
    return EXIT_FAILURE;
  }
  
}


