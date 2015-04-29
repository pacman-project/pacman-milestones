#ifndef _VISION_H_
#define _VISION_H_

#define  ARMA_DONT_USE_WRAPPER
#include <fstream>
#include <string>
#include <math.h>
#include <float.h>
#include "boost/multi_array.hpp"
#include <armadillo>
#include <Python.h>
#include <tinyxml.h>

#include <pcl/registration/transformation_estimation.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/impl/vtk_lib_io.hpp>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/registration/warp_point_rigid.h>
#include <pcl/registration/icp.h>
#include <pcl/common/pca.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

typedef pcl::PointXYZRGB ColorPointType;
typedef pcl::PointCloud<ColorPointType> ColorCloud;
typedef typename ColorCloud::Ptr ColorCloudPtr;
typedef typename ColorCloud::ConstPtr ColorCloudConstPtr;

using namespace pcl;
using namespace std;
using namespace arma;

enum Mode
{
  SAMPLE,
  LOAD,
  PARTS,
  PARTS_ALL,
  CLUSTER,
  EVALUATE,
	PART_SEG,
  EXTRACT_PATCH, 
  EXTRACT_PATCH_PTS,
	ESTIMATE_PATCH_THRESHOLD,
	ESTIMATE_PART_THRESHOLD,
	INVALID
};

struct execMode
{
	int dims;
  string patch_cls_path;
  string part_cls_path;
  Mode mode;
  string pro_path;
  string part_mats_path;
  string part_cls_mat_path;
  string part_object_occ_mat_path;
  string patch_cls_mat_path;
  string eval_path;
  string eval_object_class_path;
  string input_pcd;
  string patch_pcd;
  string patches_path;
  string input_dir;
  string train_objects_class;
	string patch_n_cls_mat;
};

struct model
{
  string id;
  vector<int> edge_pts;
  vector<double> probs;
  CloudPtr cloud_pts;
  PointCloud<pcl::Normal>::Ptr cloud_normal;
  CloudPtr cloud;
  int type;
  map<int,vector<double> > ght_rtable;
  map< uint32_t,Supervoxel<PointType>::Ptr > supervoxel_clusters;
  map<int,vector<int> > voxels_map;
  
  map<int,vector<int> > segments_dis;
  map<int,set<int> > segment_nns;
  map<int,set<int> > segments;
  map<int,CloudPtr> segment_voxels;
  map<int,PointCloud<pcl::Normal>::Ptr> segment_normals;
  map<int,pcl::Normal> segment_normal;

  map<int,set<int> > surfaces_nns;
  map<int,set<int> > surfaces;
  map<int,CloudPtr> surfaces_voxels;   
  map<int,PointCloud<pcl::Normal>::Ptr> surfaces_normals;
  map<int,pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr> surface_cvts;
  map<int,pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr> surface_border_cvts;

  map<int,Cube<double> > surface_hist;
  map<int,Cube<double> > surface_border_hist;

  map<int,vector<pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr> > surface_voxel_cvts;
  map<int,vector<Cube<double> > > surface_desc;
 
  map<int,vector<Cube<double> > > surface_voxel_hist;
  map<int,vector<Cube<double> > > surface_patch_normal;
  map<int,Cube<double> > voxel_hist;

  map<int,vector<int> > surface_voxel_map;
  map<int,vector<CloudPtr> > surface_segment_pts;

  map<int,map<int,int> > surface_segment_map;
  
  map<int,string> local_shape; 
  map<int,vector<vector<PointType> > > axis_map;

  map<int,set<int> > part_surface_map;
  map<int,Cube<double> > parts_hist;
  map<int,CloudPtr> parts; 
  map<int,set<int> > part_nns;

  // ------------------- //
  map<int,vector<Mat<double> > > trans_map;
  map<int,vector<Mat<double> > > center_map;
  map<int,vector<Cube<double> > > surface_patch_axis;
}; 

const double std_dev_pose_ = 1.;
const double std_dev_pose_local_ = 10.;
const double std_dev_normal_ = 5.;
const int max_cluster_num_ = 5;
const int pos_bins_ = 30;
const int max_cluster_iter_ = 1;
const double sector_rad_step = 5.;
const double sector_angle_step = M_PI/5.;  
const bool debug_eval_ = false;
const bool save_ = false;
const int Hist_Bin = 8;
const bool parts_ = false;
const double normal_quant = 8;
const double perc_train = 0.65;
const int min_train_size_ = 3;
const int Bin_Angle = 8;
const int Bin_Cvt = 40;
const double min_potential_ = 1e-3;
const double center_bin_ = 0;
const double normal_rad_ = 0.01;
const bool agg_thresh_min_ = true;
const double prob_thresh_ = 0.5 + 1e-2;

struct Hcluster
{
  vector<Cube<double> > memebers;
  vector<int> members_ids;
  Cube<double> center;
  Cube<double> var_diag;  
  vector<CloudPtr> mems_pts;

  vector<Cube<double> > memebers_n;
  Cube<double> center_n;
  
  Mat<double> cov_mat;
  Mat<double> cov_mat_n;

  double num_data;
  Eigen::MatrixXd cov;
  Eigen::MatrixXd mu;
  double prior;
	double dist_threshold;
};

class vision{
  public:

	bool dims_meter_;
  int model_size_;
  string file_name_;

  vision();
  ~vision();

  wchar_t *nstrws_convert(const char *raw) {
  	wchar_t *rtn = (wchar_t *) calloc(1, (sizeof(wchar_t) * (strlen(raw) + 1)));
    setlocale(LC_ALL,"en_US.UTF-8");
    mbstowcs(rtn, raw, strlen(raw));
    return rtn;
	}

	CloudPtr load_cloud(string file_name);
  PointCloud<pcl::Normal>::Ptr estimate_normal();
  Eigen::Vector3f get_cloud_center(CloudPtr cloud); 
  CloudPtr down_sample(CloudPtr cloud,double down_sample_size);
  vector<int> get_points_from_visualizer(const pcl::visualization::AreaPickingEvent &area);  
  void segment_supervoxel(model &cur_model);
  void superpixel_clustering(model &cur_model,double epsilon=0.1);


  model decompose_object(CloudPtr cloud);
  vector<string> get_all_files(string dir_name,string pattern=".pcd");
  void extract_feature(model &m);

  void normalize_hist(Cube<double> &hist);
  void normalize_hist_gaus(Cube<double> &hist);

  double chi_hist_distance(Cube<double> hist1,Cube<double> hist2);
  double bha_hist_distance(Cube<double> hist1,Cube<double> hist2);
  double euc_hist_distance(Cube<double> hist1,Cube<double> hist2);
  vector<Hcluster> cluster_features(vector<model> models);
  double agg_clustering(vector<model> models,vector<Hcluster> &clusters);
  double inter_hist_distance(Cube<double> hist1,Cube<double> hist2);
  double dist_cluster(Hcluster cluster1,Hcluster cluster2,Mat<double> dist_mat);

  Cube<double> histogram_from_curvature(pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cvts);
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr estimate_curvature(PointCloud<pcl::Normal>::Ptr cloud_normal,CloudPtr cloud);
  void normalize_curvature(pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &cvts);
  vector<string> sample_data(vector<string> files,double percentage);
  double dist_cluster_avg(Hcluster cluster1,Hcluster cluster2);
  double dist_cluster_max(Hcluster cluster1,Hcluster cluster2);
  void get_cluster_threshold();

  void label_parts(const pcl::visualization::AreaPickingEvent &area);
  vector<model> read_prototypes(string path,bool is_prototype=true);
  vector<Hcluster> read_clusters(vector<string> dirs,vector<model> &models_all);
  void save_feature(model m);


  void make_code_book(vector<Hcluster> &clusters);

  vector<int> measure_similarity_codebook(vector<Hcluster> clusters,Cube<double> hist);
  void estimate_codebook_frequency(vector<Hcluster> clusters);
  model make_evaluation_model(string eval_path);
  vector<model> make_evaluation_models(string eval_path);

	void extract_feature_patch(model &m);
	void extract_feature_axis(model &m);
	void estimate_co_occurance(vector<model> models,vector<model> pro_models,vector<Hcluster> clusters);
	model refine_models(model m);
	vector<set<int> > evaluate_model_patch(model m,vector<Hcluster> clusters);
	vector<set<int> > evaluate_model_patch(model m,vector<Hcluster> clusters,vector<set<int> > regions);   
	void cluster_surface_patch(model &cur_model);
	vector<Hcluster> read_clusters_patch(vector<string> paths,vector<model> &models_all);

	model read_model(vector<string> files,bool is_prototype=true);

	void extract_patch(CloudPtr cloud,string file_name);
	set<int> refine_patch_points( vector<PointType> pts,CloudPtr cloud,model &m,Mat<double> &dist,map<int,int> &sid_map,int c_id );
	ColorCloudPtr save_prototype(model m,set<int> surf_vec);
	vector<CloudPtr> extract_part_pts(model m,vector<set<int> > regs);
	double estimate_overlap(vector<CloudPtr> parts,model m);
	void visualize_surfaces(model m);
	vector<set<int> > refine_multiple_patches(vector<CloudPtr> clouds,CloudPtr cloud,model &m);
	vector<set<int> > evaluate_model_patch_new(model m,vector<Hcluster> clusters,vector<set<int> > regs);
	void convert_to_csv(CloudPtr cloud,PointCloud<pcl::Normal>::Ptr cloud_normal);   

	void visualize_axis(CloudPtr cloud,vector<vector<PointType> > axis_list);    
	vector<vector<PointType> > get_axis(CloudPtr cloud,PointCloud<pcl::Normal>::Ptr cloud_normal);
	void init_axis();
  
	double shape_context_distance(Cube<double> hist1,Cube<double> hist2);  
	double dist_cluster_avg(Hcluster cluster1,Cube<double> desc);
	void make_parts_visualization(vector<vector<set<int> > > parts_h,vector<vector<set<int> > > parts_h_pts,model m);
	void part_codebook_visualization(model m);
	vector<set<int> > recognize_parts(model m,vector<Hcluster> clusters);

	void read_input();
	void execution();

	void estimate_threshold_protoypes();
	void estimate_part_threshold_protoypes();

	void sample_input_data();
	void load_data();
	void get_parts();
	void get_all_parts();
	void do_clustering();
	void do_evaluation();
	void do_parts_segmentation();
	void do_patch_extraction();
	void do_patch_extraction_pts();

	model refine_models(vector<model> models,vector<set<int> > &parts);
	
	vector<double> likelihoods_;
	map<int,vector<double> > ght_rtable_;

	vector<vector<int> > ids_;
	vector<vector<double> > normals_model_map_;
	vector<vector<double> > poses_model_map_;

	map<int,vector<double> > voxels_normals_model_map_;
	double voxel_changed_;
	set<int> seen_voxels_;
	vector<double> cluster_likelihoods_;
	set<int> cur_seen_voxels_;
	vector<CloudPtr> patches_labeled_;
	bool dims_meter;
	vector<model> models_;
	double min_cvt_x, min_cvt_y, min_cvt_z;
	double max_cvt_x, max_cvt_y, max_cvt_z;

	map<int,int> model_map;
	map<int,int> model_local_map;
	vector<Cube<double> > hists; 
	vector<Cube<double> > hists_border; 

	double threshold_;
	vector<double> pair_dists_;

	vector<double> pair_dists_patch_;

	int id_;
	map<string,vector<model> > model_prototypes_;  
	map<string,vector<double> > prototype_frequency_;
	map<string,vector<double> > prototype_occ_;

	map<string,double> prototype_rel_scale_;

	Mat<double> dist_prototype_;
	vector<model> eval_models;
	map<int,string> cluster_name_map_;
	int k_cls_; 
	double min_threshold_;

	Mat<double> occ_mat_;
	Mat<double> occ_mat_n_;
	Cube<double> occ_mat_tri_;
	map<int,Mat<double> > occ_mat_parts_;
	map<int,vec> occ_cls_parts_;
	map<int,Mat<double> > occ_mat_parts_cls_;
	Mat<double> patch_occ_n_;

	bool patch_;
	bool angle_;
	double min_angle_;
	Mat<double> dist_mat_;
	double potential_;
	PyObject *pName, *pModule;
	int parts_axis_call_;
	double ind_max;
	int id_cls_;
	int cls_max_;
	map<int,string> object_cls_;
	map<int,int> object_cls_map_;
	bool parts_cls_;
	bool object_ins_cls_;
	int border_patches_;
	int border_parts_;
	CloudPtr cloud_; 
	double min_dist_cls_;
	vector<vector<set<int> > > parts_h_;
	vector<vector<set<int> > > parts_h_pts_;
	map<int,int> region_id_;
	map<int,int> region_id_pre_;
	map<int,vector<int> > patch_cb_;
	map<int,vector<int> > part_cb_;
	vector<double> probs_;
	vector<Hcluster> part_clusters_;
	vector<Hcluster> patch_clusters_; 
	execMode exec_mode_;
	map<string,Mode> mode_map_;
	Mat<double> part_object_freq_;
	double min_dist_val_;
	double max_dist_val_;
	bool agg_max_;
	string default_path_;
  double voxel_resolution;
  double seed_resolution;
	vector<double> dists_cb_;
	vector<double> dists_cb_all_;
	vector<double> regs_prob_;
	vector<int> ids_all_;	
	double region_threshold_;
	vector<Mat<double> > dist_mats_;
	int table_size_;
		
	map<int,vector<set<int> > > regs_all_;
	map<int,double> regs_potential_;
	vector<double> p_overlap_;
	vector<double> p_ids_;
	set<int> ids_seg_;

  map<int,int> model_id_;
  map<int,int> part_id_;
	bool debug_;
}; 
#endif

