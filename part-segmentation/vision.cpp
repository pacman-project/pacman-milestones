#include "vision.h"

vision::vision()
{
	region_threshold_ = 1e-5;
  object_ins_cls_ = false;
  parts_cls_ = false;
  parts_axis_call_ = 0;
  patch_ = true;
  angle_ = false;
  threshold_ = FLT_MAX;
  min_cvt_x = FLT_MAX; min_cvt_y = FLT_MAX; min_cvt_z = FLT_MAX;
  max_cvt_x = FLT_MIN; max_cvt_y = FLT_MIN; max_cvt_z = FLT_MIN;
	max_dist_val_ = FLT_MIN;
	min_dist_val_ = FLT_MAX;
	agg_max_ = false;
}
//-------------------------------------------------------------------
vision::~vision()
{
}

CloudPtr vision::down_sample(CloudPtr cloud,double down_sample_size)
{
  CloudPtr cloud_filtered(new Cloud());
  VoxelGrid<PointType> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(down_sample_size,down_sample_size,down_sample_size);
  vox.filter(*cloud_filtered);
  return cloud_filtered;
}

CloudPtr vision::load_cloud(string file_name)
{
  pcl::PCDReader reader;
  CloudPtr cloud (new Cloud);
  reader.read (file_name, *cloud);

  if( debug_ ){
	cout << "part segmentation" << endl;
	string input = "y";
	do{
		input = "y";
		cout << "select seg" << endl;
		pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("cloud viewer");
		boost::function< void (const pcl::visualization::AreaPickingEvent &)> f = boost::bind (&vision::get_points_from_visualizer, this, _1);
		viewer->registerAreaPickingCallback(f);
		viewer->addPointCloud(cloud);

		while(!viewer->wasStopped())
		{
			viewer->spinOnce (100);
		}
		set<int>::iterator its;
		vector<int> inds;
		for( its = ids_seg_.begin(); its != ids_seg_.end(); its++ )
			inds.push_back(*its);
		ids_seg_.clear();
		ids_.push_back(inds);
		if( inds.size() > 0 ){
			cout << "are you done? " << endl;
			cin >> input;
		}
	}while(input != "y" );
  }

  string root = file_name.substr(file_name.find_last_of("/")+1);
  root  = root.substr(0,root.find("."));
  file_name_ = root;

// ** refinement ** //
  vector<vector<int> > ids;
  ids = ids_;
// ** //


	string dir_patch_name;
  if( ids_.size() == 0 )
    return cloud;
	else
	{
		dir_patch_name = default_path_ + "patches/";
		boost::filesystem::path dir_patch(dir_patch_name);
		boost::filesystem::create_directories(dir_patch);
	}
  ColorCloudPtr cloud_deb(new ColorCloud);
  for( size_t i = 0; i < ids.size(); i++ )
  {
    double r = rand() %255; double g = rand() %255; double b = rand() %255;
    for( size_t j = 0; j < ids[i].size(); j++ )
    {
      int id = ids[i][j];
      ColorPointType p;
      p.x = cloud->points[id].x; p.y = cloud->points[id].y; p.z = cloud->points[id].z;
      p.r = r; p.b = b; p.g = g;
      cloud_deb->points.push_back(p);
    }
  }
  
  if( debug_ ){
  pcl::visualization::PCLVisualizer *viewer_ = new pcl::visualization::PCLVisualizer("cloud viewer");
  viewer_->addPointCloud(cloud,"cloud_orig");
  viewer_->addPointCloud(cloud_deb,"cloud_deb");
  while(!viewer_->wasStopped())
  {
    viewer_->spinOnce (100);
  }
  }

	string dir_cur_name = dir_patch_name + file_name_ + "/";
  boost::filesystem::path dir_cur(dir_cur_name);
  boost::filesystem::create_directories(dir_cur);

  cloud_deb->width = 1;
  cloud_deb->height = cloud_deb->points.size();
  //string patch_file = dir_patch_name + root + "_patches.pcd";
	string patch_file = dir_cur_name + root + "_patches.pcd";
	cout << "to store in " << patch_file << endl;
  //pcl::io::savePCDFileASCII (patch_file, *cloud_deb);

  for( size_t i = 0; i < ids.size(); i++ )
  {  
    if( ids[i].size() < 5 )
      continue;
    ColorCloudPtr cloud_deb_local(new ColorCloud);
    double r = rand()%255; double g = rand()%255; double b = rand()%255;
    for( size_t j = 0; j < ids[i].size(); j++ )
    {
      int id = ids[i][j];
      ColorPointType p;
      p.x = cloud->points[id].x; p.y = cloud->points[id].y; p.z = cloud->points[id].z;
      p.r = r; p.b = b; p.g = g;
      cloud_deb_local->points.push_back(p);
    }
   /* pcl::visualization::PCLVisualizer *viewer_local = new pcl::visualization::PCLVisualizer("cloud viewer");
    viewer_local->addPointCloud(cloud_deb_local);
    while(!viewer_local->wasStopped())
    {
      viewer_local->spinOnce (100);
    }*/
    cloud_deb_local->width = 1;
    cloud_deb_local->height = cloud_deb_local->points.size();
    stringstream ss;
		ss << dir_cur_name << root << "_patch" << i << ".pcd";
    pcl::io::savePCDFileASCII (ss.str(), *cloud_deb_local);
  }
  ids_.clear(); 
  return cloud;
}


vector<set<int> > vision::refine_multiple_patches(vector<CloudPtr> clouds,CloudPtr cloud,model &m)
{ 
  cout << "in refine patches" << endl;
  m = decompose_object(cloud);
  extract_feature(m);
  extract_feature_patch(m);
  extract_feature_axis(m);

	int count = 0;
	map<int,int> sid_map;
	map<int,CloudPtr>::iterator it;
	for( it = m.surfaces_voxels.begin(); it != m.surfaces_voxels.end(); it++,count++ )
		sid_map[(*it).first] = count;
	Mat<double> dist(clouds.size(),m.surfaces_voxels.size());
	dist.fill(FLT_MAX);

  cout << "to refine patches" << endl;
  vector<set<int> > surfs(clouds.size());
  for( size_t ic = 0; ic < clouds.size(); ic++ )
  {
    vector<PointType> pts;
    cout << "to refine " << ic << endl;
    for( size_t i = 0; i < clouds[ic]->points.size(); i++ )
      pts.push_back(clouds[ic]->points[i]);
    set<int> surf_vec = refine_patch_points(pts,cloud,m,dist,sid_map,ic);
  //  surfs.push_back(surf_vec);
  }
	 
	for( int c = 0; c < dist.n_cols; c++ )
	{
		double min_dist = FLT_MAX;
		int min_id = -1;
		for( int r = 0; r < dist.n_rows; r++ )
		{
			if( dist(r,c) < min_dist )
			{
				min_dist = dist(r,c);
				min_id = r;		
			}
		}
		if( min_id >= 0 )
			surfs[min_id].insert(c);		
	}
  return surfs;  
}

set<int> vision::refine_patch_points( vector<PointType> pts,CloudPtr cloud,model &m,Mat<double> &dist,map<int,int> &sid_map,int c_id )
{
  map<int,CloudPtr>::iterator it;
  set<int> surf_vec;
  for( size_t i = 0; i < pts.size(); i++ )
  {
    //double min_dist = FLT_MAX;
		double min_dist = 0.03;
    int min_id = -1;
    for( it = m.surfaces_voxels.begin(); it != m.surfaces_voxels.end(); it++ )
    {
      search::KdTree<PointType>::Ptr tree (new search::KdTree<PointType>);
      CloudPtr cloud_cur = (*it).second;
      tree->setInputCloud(cloud_cur);
      vector< int > k_indices; vector< float > k_sqr_distances;
      int k = 2;
      tree->nearestKSearch(pts[i],k,k_indices,k_sqr_distances);
      //if( k_sqr_distances[0] < min_dist ) 
			if( ( k_sqr_distances[0] < min_dist ) && ( k_sqr_distances[0] < dist(c_id,sid_map[(*it).first]) ) )
      {
				dist(c_id,sid_map[(*it).first]) = k_sqr_distances[0];
       /* min_dist = k_sqr_distances[0];
        min_id = (*it).first;*/
      }
    }
    //if( min_id >= 0 ){
			//cout << c_id << ": " << dist.n_rows << " : " << sid_map[(*it).first] << " : " << dist.n_cols << endl;
      //surf_vec.insert(min_id);
   // }
  }
  return surf_vec;
}

ColorCloudPtr vision::save_prototype(model m,set<int> surf_vec)
{
  string cname = file_name_;
  if( cname.find("patch") != string::npos )
  {
    string cue = "patch";
    string fname = cname.substr(0,cname.find(cue));
    string lname = cname.substr(cname.find(cue)+cue.length());
    cname = fname + "reg" + lname;
  }

  //string dir_patch_name = "/media/TOSHIBA\ EXT/prototypes/";
	string dir_patch_name = default_path_ + "prototypes/";
  boost::filesystem::path dir_patch(dir_patch_name);
  boost::filesystem::create_directories(dir_patch);

  ColorCloudPtr cloud_deb(new ColorCloud);
  map<int,CloudPtr>::iterator it_sv;
  set<int>::iterator its;
	double r = rand() % 255; double g = rand() % 255; double b = rand() % 255;
  for( its = surf_vec.begin(); its != surf_vec.end(); its++ )
  {
    //double r = rand() % 255; double g = rand() % 255; double b = rand() % 255;
    CloudPtr cloud_cur = m.surfaces_voxels[*its];
    for( size_t i = 0; i < cloud_cur->points.size(); i++ )
    {
      ColorPointType p;
      p.r = r; p.g = g; p.b = b;
      p.x = cloud_cur->points[i].x; p.y = cloud_cur->points[i].y; p.z = cloud_cur->points[i].z;
      cloud_deb->points.push_back(p);
    }
    pcl::PointCloud<PointNormal>::Ptr cloud_ap (new pcl::PointCloud<PointNormal>);
    PointCloud<pcl::Normal>::Ptr cloud_np = m.surfaces_normals[*its];
    PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_cvp = m.surface_cvts[*its];
    for( size_t i = 0; i < cloud_cur->points.size(); i++ )
    {
      PointNormal point;
      point.x = cloud_cur->points[i].x; point.y = cloud_cur->points[i].y; point.z = cloud_cur->points[i].z;
      point.normal_x = cloud_np->points[i].normal_x; point.normal_y = cloud_np->points[i].normal_y; point.normal_z = cloud_np->points[i].normal_z;
      cloud_ap->points.push_back(point);
    }
    if( ( cloud_ap->points.size() == 0 ) || ( cloud_cvp->points.size() == 0 ) )
      continue;
    stringstream ss;
    ss << dir_patch_name << cname << "_part" << "_" << *its << ".pcd";
    cloud_ap->width = 1;
    cloud_ap->height = cloud_ap->points.size();
    pcl::io::savePCDFileASCII (ss.str(),*cloud_ap);
    stringstream ssv;
    cloud_cvp->width = 1;
    cloud_cvp->height = cloud_cvp->points.size();
    ssv << dir_patch_name << cname << "_cvt" << "_" << *its << ".pcd";
    pcl::io::savePCDFileASCII (ssv.str(),*cloud_cvp);
    vector<pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr> voxel_cvts = m.surface_voxel_cvts[*its];
    for( size_t i = 0; i < voxel_cvts.size(); i++ )
    {
      PointCloud<pcl::PrincipalCurvatures>::Ptr cvt_voxel = voxel_cvts[i];
      if( cvt_voxel->points.size() == 0 )
        continue;
      cvt_voxel->width = 1;
      cvt_voxel->height = cvt_voxel->points.size();
      stringstream ss_v;
      ss_v << dir_patch_name << cname << "_cvt_voxel" << *its << "_" << i << ".pcd";
      pcl::io::savePCDFileASCII (ss_v.str(),*cvt_voxel);

      CloudPtr cloud_patch(new Cloud);
      Cube<double> cube_patch = m.surface_patch_normal[*its][i];
      for( int ip = 0; ip < cube_patch.n_elem; ip++ )
      {
        PointType p;
        p.x = cube_patch(ip); p.y = 0; p.z = 0;
        cloud_patch->points.push_back(p);
      }
      if( cloud_patch->points.size() == 0 )
        continue;
      cloud_patch->width = 1;
      cloud_patch->height = cloud_patch->points.size();
      stringstream ss_p;
      ss_p << dir_patch_name << cname  << "_normal_patch" << *its << "_" << i << ".pcd";
      pcl::io::savePCDFileASCII (ss_p.str(),*cloud_patch);

      CloudPtr cloud_axis(new Cloud);
      Cube<double> cube_axis = m.surface_patch_axis[*its][i];
      for( int ip = 0; ip < cube_axis.n_elem; ip++ )
      {
        PointType p;
        p.x = cube_axis(ip); p.y = 0; p.z = 0;
        cloud_axis->points.push_back(p);
      }
      if( cloud_axis->points.size() == 0 )
        continue;
      cloud_axis->width = 1;
      cloud_axis->height = cloud_axis->points.size();
      stringstream ss_ax;
      ss_ax << dir_patch_name << cname << "_axis_patch" << *its << "_" << i << ".pcd";
      pcl::io::savePCDFileASCII (ss_ax.str(),*cloud_axis);

      CloudPtr cloud_voxel = m.segment_voxels[m.surface_voxel_map[*its][i]];
      if( cloud_voxel->points.size() == 0 )
        continue;

      cloud_voxel->width = 1; cloud_voxel->height = cloud_voxel->points.size();
      stringstream ss_vo;
      ss_vo << dir_patch_name << cname  << "_patch" << *its << "_" << i << ".pcd";
      pcl::io::savePCDFileASCII (ss_vo.str(),*cloud_voxel);
    }

  }

	if( surf_vec.size() > 0 ){
		stringstream ss;
		//ss << dir_patch_name << file_name_ << ".pcd";
    ss << dir_patch_name << cname << ".pcd";		
		m.cloud->width = 1;
		m.cloud->height = m.cloud->points.size();
		pcl::io::savePCDFileASCII (ss.str(),*m.cloud);
	}
	if(debug_eval_){
  pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("extracted surfaces");
  viewer->addPointCloud(cloud_deb,"debug");
//  viewer->addPointCloud(cloud,"orig");

  while(!viewer->wasStopped())
  {
    viewer->spinOnce (100);
  }
  delete viewer;

}
  return cloud_deb;
}

void vision::init_axis()
{
  string module = "AJ_axis_Gaussian";
  
  Py_Initialize();
  wchar_t **rtn = (wchar_t **) calloc(2, sizeof(wchar_t *));
  rtn[0] = nstrws_convert(module.c_str());

  PySys_SetArgv(1, rtn);
  pName = PyUnicode_FromFormat(module.c_str());
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);
  if( !pModule )
  {
    PyErr_Print();
    cout << "failed to load: " << module << endl;
  }

}

vector<vector<PointType> > vision::get_axis(CloudPtr cloud,PointCloud<pcl::Normal>::Ptr cloud_normal)
{ 
  vector<vector<PointType> > axis_list  ;
  cout << "to load" << endl;
  PyObject *pArgs, *pValue, *pFunc;
  pFunc = PyObject_GetAttrString(pModule,"axis");
  if (pFunc && PyCallable_Check(pFunc))
  {
    pArgs = PyTuple_New(2);
    convert_to_csv(cloud,cloud_normal);
    pValue = PyUnicode_FromFormat("/tmp/cloud.csv");

    PyTuple_SetItem(pArgs, 0, pValue);
    pValue = PyLong_FromLong(0);
    PyTuple_SetItem(pArgs, 1, pValue);
    pValue = PyObject_CallObject(pFunc, pArgs);
    for( int i = 0; i < 3; i++ )
    {
      PyObject *item,*obj_temp,*sitem;
      obj_temp = PySequence_GetItem(pValue, i);
      vector<PointType> vals_cur;
      for( int j = 0; j < 3; j++ )
      {
        item = PySequence_GetItem(obj_temp, j);
        PointType p;
        for( int k = 0; k < 3; k++ )
        {
          sitem = PySequence_GetItem(item, k);
          double val = PyFloat_AsDouble(sitem);
          if( k == 0 )
            p.x = val;
          else if( k == 1 )
            p.y = val;
          else
            p.z = val;
        }
        vals_cur.push_back(p);
      }
      axis_list.push_back(vals_cur);
    }
    PyObject *item;
    item = PySequence_GetItem(pValue, 3);
    ind_max = PyFloat_AsDouble(item);
  }
  else
  {
    cout << "cannot find function" << endl;
  }

  return axis_list;
}

void vision::convert_to_csv(CloudPtr cloud,PointCloud<pcl::Normal>::Ptr cloud_normal)
{
  ofstream ofs("/tmp/cloud.csv");
  for( size_t i = 0; i < cloud->points.size(); i++ )
  {
    stringstream ss;
    ofs << cloud->points[i].x  << ", "  << cloud->points[i].y  << ", " << cloud->points[i].z  << ", "
       << cloud_normal->points[i].normal_x  << ", " << cloud_normal->points[i].normal_y  << ", " << cloud_normal->points[i].normal_z << ", " << cloud_normal->points[i].curvature ;
    ofs << endl;    
  }
  ofs.close();
}

void vision::visualize_axis(CloudPtr cloud,vector<vector<PointType> > axis_list)
{
  pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("axis viewer");
  viewer->addPointCloud(cloud,"orig");
  for( size_t i = 0; i < axis_list.size(); i++ )
  {
    double r,g,b;
    if( i == 0 )
    {
      r = 255; g = 0; b = 0;
    }
    else if( i == 1 )
    {
      r = 0; g = 255; b = 0;
    }
    else
    {
      r = 0; g = 0; b = 255;
    }

    vector<PointType> cur_vals = axis_list[i];
    stringstream ss1;
    ss1 << "line"  << i << 0;
    Eigen::Vector3f center_vec = get_cloud_center(cloud);
    PointType center;
    center.x = center_vec(0);  center.y = center_vec(1);  center.z = center_vec(2);
    viewer->addLine(cur_vals[0],cur_vals[1],r,g,b,ss1.str());
    Eigen::Vector3f vec_1,vec_2;
    vec_1(0) = cur_vals[0].x; vec_1(1) = cur_vals[0].y; vec_1(2) = cur_vals[0].z;
    vec_2(0) = cur_vals[1].x; vec_2(1) = cur_vals[1].y; vec_2(2) = cur_vals[1].z;
    cout << "vector " << i << (vec_2 - vec_1).transpose() << endl ;
  }

  while(!viewer->wasStopped())
  {
    viewer->spinOnce (100);
  }
  delete viewer;

}

void vision::extract_patch(CloudPtr cloud,string path)
{
  vector<PointType> pts;
  vector<double> vals;
  ifstream ifs(path.c_str());
  if( ifs.is_open() )
  {
    string str;
    while( ifs.good() )
    {
      getline(ifs,str);
      if( str.length() <= 1 )
        continue;
      stringstream ss(str);
      double val;
      ss >> val;
      vals.push_back(val);
    }
  }
 
  vector<vector<double> > quats_vals;
  double w,x,y,z;
  for( size_t i = 0; i < vals.size(); i+=7 )
  {
    PointType point;
    vector<double> quat_val;
    for( size_t j = 0; j < 7; j++ )
    {
      switch(j)
      {
        case 0: 
          point.x = vals[i+j];
        case 1: 
          point.y = vals[i+j]; 
        case 2: 
          point.z = vals[i+j];   
        case 3:    
          quat_val.push_back(vals[i+j]);
        case 4:
          quat_val.push_back(vals[i+j]);
        case 5:
          quat_val.push_back(vals[i+j]);
        case 6:
          quat_val.push_back(vals[i+j]);
      }
    }
    quats_vals.push_back(quat_val);
    pts.push_back(point);
  }
  
  vector<PointType> pts_end;
  for( size_t i = 0; i < pts.size(); i++ )
  {
    Eigen::Quaternionf quat(quats_vals[i][0],quats_vals[i][1],quats_vals[i][2],quats_vals[i][3]);
    Eigen::Matrix3f rot_mat = quat.toRotationMatrix();
    Eigen::Vector3f vec_cur;
    vec_cur(0) = pts[i].x; vec_cur(1) = pts[i].y; vec_cur(2) = pts[i].z; 
    Eigen::Vector3f new_vec = rot_mat * vec_cur; 
    Eigen::Vector3f trans;
    trans(0) = 0; trans(1) = 0; trans(2) = -0.05;
    trans = rot_mat * trans;
    new_vec = vec_cur + trans;
    PointType pt_end;
    pt_end.x = new_vec(0); pt_end.y = new_vec(1); pt_end.z = new_vec(2);
    pts_end.push_back(pt_end);
  }
  model m;
  m = decompose_object(cloud);
  extract_feature(m);
  extract_feature_patch(m);

  PointType pt;
  pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("extracted surfaces");
  viewer->addPointCloud(cloud,"orig");
  viewer->addSphere(pts[0],0.01,255,0,0,"s1");
  pt = pts_end[0];
  viewer->addLine(pts[0],pt,255,0,0,"arrow1");
  viewer->addSphere(pts[1],0.01,255,0,0,"s2");
  pt = pts_end[1];
  viewer->addLine(pts[1],pt,255,0,0,"arrow2");
  viewer->addSphere(pts[2],0.01,255,0,0,"s3");
  pt = pts_end[2];
  viewer->addLine(pts[2],pt,255,0,0,"arrow3");
  while(!viewer->wasStopped())
  {
    viewer->spinOnce (100);
  }
  delete viewer;
}

vector<int> vision::get_points_from_visualizer(const pcl::visualization::AreaPickingEvent &area)
{
  cout << "area captured" << endl;
  vector<int> indices;
  area.getPointsIndices(indices);
  cout << "area size is: " << indices.size() << endl;
  //ids_.push_back(indices);

	for( size_t i = 0; i < indices.size(); i++ )
		ids_seg_.insert(indices[i]);
  return indices;
}

PointCloud<pcl::Normal>::Ptr vision::estimate_normal()
{
  cout << "in estimate normals" << endl;
  NormalEstimation<PointType, pcl::Normal> ne;
  ne.setInputCloud(cloud_);
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
  ne.setSearchMethod(tree);
  PointCloud<pcl::Normal>::Ptr cloud_normals (new PointCloud<pcl::Normal>);
  if( dims_meter_ )
    ne.setRadiusSearch (5.0);
  else 
   // ne.setRadiusSearch (0.05);
    ne.setRadiusSearch (normal_rad_);
  ne.setSearchSurface(cloud_);
  ne.compute (*cloud_normals);
  cout << "normals computed " << endl;
  return cloud_normals;
}


Eigen::Vector3f vision::get_cloud_center(CloudPtr cloud)
{
  Eigen::Vector4f centroid;
  compute3DCentroid(*cloud,centroid);
  return centroid.block(0,0,3,1);
}


void vision::segment_supervoxel(model &cur_model)
{
  cout << "to segment supervoxel" << endl;
	cout << voxel_resolution << " : " << seed_resolution << endl;
  /*double spatial_importance = 10;
  double normal_importance = 7;
*/
  double spatial_importance = 1.0;
  double normal_importance = 0.7;	
  pcl::SupervoxelClustering<PointType> super(voxel_resolution, seed_resolution,false);
  super.setInputCloud (cur_model.cloud);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);
  map< uint32_t,Supervoxel<PointType>::Ptr > supervoxel_clusters;
  super.extract(supervoxel_clusters);
  
  pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud_deb(new pcl::PointCloud< pcl::PointXYZRGBA >);
  cloud_deb = super.getColoredCloud();
  multimap<uint32_t, uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);
  multimap<uint32_t, uint32_t>::iterator it;
  map<int,vector<int> > voxels_map;

	for( it = supervoxel_adjacency.begin(); it != supervoxel_adjacency.end(); it++ )
	{
		uint32_t supervoxel_label = it->first;
		std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
		for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
			voxels_map[supervoxel_label].push_back(adjacent_itr->second);
	}

  cur_model.supervoxel_clusters = supervoxel_clusters;
  cur_model.voxels_map = voxels_map;
  cout << "to cluster superpixel" << endl;
	superpixel_clustering(cur_model,0.03);
  cout << "to cluster surface patch" << endl;
  cluster_surface_patch(cur_model);
}

void vision::superpixel_clustering(model &cur_model,double epsilon)
{
  double min_bound = 1. - epsilon;
  double max_bound = 1. + epsilon;
  map<int,int> voxel_segment_map;
  map<int,set<int> > sv_cls;
  map< uint32_t,Supervoxel<PointType>::Ptr >::iterator it_sc; 
  map< uint32_t,Supervoxel<PointType>::Ptr > supervoxel_clusters = cur_model.supervoxel_clusters;

	int min_pts = 10;
	vector<set<int> > segs;
	set<int> rejs;
	for( it_sc = supervoxel_clusters.begin(); it_sc != supervoxel_clusters.end(); it_sc++ )
	{
		if( supervoxel_clusters[(*it_sc).first]->voxels_->points.size()  < min_pts )
		{
			rejs.insert((*it_sc).first);
		}
		else
		{
			set<int> seg;
			seg.insert((*it_sc).first);
			segs.push_back(seg);
		}
	}	
		
//	cout << "#of rejected: " << rejs.size() << endl;

	bool changed = true;
	while(changed)
	{
		set<int> seen_ids;
		vector<set<int> > segs_cur;
		for( size_t i = 0; i < segs.size(); i++ )
		{
			if( seen_ids.find(i) != seen_ids.end() )
				continue;
			set<int> seg_cur;
			seg_cur.insert(i);
			set<int>::iterator its;
			Eigen::Vector3f normal_ref_vec;
			normal_ref_vec.fill(0);
			set<int> nns;
			for( its = segs[i].begin(); its != segs[i].end(); its++ )
			{
				int id = (*its);
				normal_ref_vec(0) += supervoxel_clusters[id]->normal_.normal_x;
				normal_ref_vec(1) += supervoxel_clusters[id]->normal_.normal_y;
				normal_ref_vec(2) += supervoxel_clusters[id]->normal_.normal_z;

				for( size_t j = 0; j < cur_model.voxels_map[*its].size(); j++ )
				{
					int n_id = cur_model.voxels_map[*its][j];
					if( ( segs[i].find(n_id) == segs[i].end() ) && ( rejs.find(n_id) == rejs.end() ) )
						nns.insert(n_id);
				}
			}
			normal_ref_vec /= (double) segs[i].size(); 

			for( size_t j = i+1; j < segs.size(); j++ )	
			{
				if( seen_ids.find(j) != seen_ids.end() )
					continue;
				bool valid = false;
				Eigen::Vector3f normal_cur_vec;
				normal_cur_vec.fill(0);
				for( its = segs[j].begin(); its != segs[j].end(); its++ )
				{
					if( nns.find(*its) != nns.end() )
						valid = true;

	        int id = (*its);
		      normal_cur_vec(0) += supervoxel_clusters[id]->normal_.normal_x;
			    normal_cur_vec(1) += supervoxel_clusters[id]->normal_.normal_y;
				  normal_cur_vec(2) += supervoxel_clusters[id]->normal_.normal_z;		
				}
	      normal_cur_vec /= (double) segs[j].size(); 
        double val = normal_ref_vec.dot(normal_cur_vec);
        val /= (normal_cur_vec.norm() * normal_ref_vec.norm());
        //if( ( fabs(val) <= ( max_bound ) ) && ( fabs(val) >= ( min_bound ) )  && (valid) ) 
				if( fabs(val) >= ( min_bound )&& (valid)) 
				{
					seen_ids.insert(j);
					seg_cur.insert(j);
        }

			}
			set<int> svs;
			for( its = seg_cur.begin(); its != seg_cur.end(); its++ )
			{
				set<int>::iterator its2;
				for( its2 = segs[*its].begin(); its2 != segs[*its].end(); its2++ )
					svs.insert(*its2);
			}
			segs_cur.push_back(svs);
			//segs_cur.push_back(seg_cur); 
		}


		if( segs_cur.size() == segs.size() )
			changed = false;
		else
			segs = segs_cur;
	}

	ColorCloudPtr cloud_deb(new ColorCloud);	
	for( size_t i = 0; i < segs.size(); i++ )
	{
		double r = rand() % 255; double g = rand() % 255; double b = rand() % 255;
		cur_model.segments[i] = segs[i];
		set<int> segs_nns;
		set<int> nns;
		set<int>::iterator its;
		pcl::Normal normal_ref;
		normal_ref.normal_x = 0; normal_ref.normal_y = 0; normal_ref.normal_z = 0;
		CloudPtr cloud_cur(new Cloud);
		PointCloud<pcl::Normal>::Ptr normals_cur(new PointCloud<pcl::Normal>);

    for( its = segs[i].begin(); its != segs[i].end(); its++ )
    {
			int id = (*its);
      normal_ref.normal_x += supervoxel_clusters[id]->normal_.normal_x;
      normal_ref.normal_y += supervoxel_clusters[id]->normal_.normal_y;
      normal_ref.normal_z += supervoxel_clusters[id]->normal_.normal_z;
      for( size_t j = 0; j < cur_model.voxels_map[*its].size(); j++ )
      {
        int n_id = cur_model.voxels_map[*its][j];
        if( ( segs[i].find(n_id) == segs[i].end() ) &&  ( rejs.find(n_id) == rejs.end() ) )
          nns.insert(n_id);
      }

			CloudPtr voxels = supervoxel_clusters[id]->voxels_;
			PointCloud<pcl::Normal>::Ptr normals = supervoxel_clusters[id]->normals_;
			for( size_t j = 0; j < voxels->points.size(); j++ ){
				cloud_cur->points.push_back(voxels->points[j]);
				normals_cur->points.push_back(normals->points[j]);

				ColorPointType p;
				p.x = voxels->points[j].x; p.y = voxels->points[j].y; p.z = voxels->points[j].z;
				p.r = r; p.g = g; p.b = b;
				cloud_deb->points.push_back(p);
			}
    }
		normal_ref.normal_x /= segs[i].size(); 
    normal_ref.normal_y /= segs[i].size();
    normal_ref.normal_z /= segs[i].size();
		cur_model.segment_normal[i] = normal_ref;

		for( size_t j = 0; j < segs.size(); j++ )	
		{
			if( j == i )
				continue;
			set<int>::iterator its2;
			bool valid = false;
			for( its2 = segs[j].begin();( (its2 != segs[j].end()) && (!valid)); its2++ )
			{	
				if( nns.find(*its2) != nns.end() )
					valid = true;
			}
			//if( nns.find(j) != nns.end() )
			if(valid)
				segs_nns.insert(j);
		}
		cur_model.segment_nns[i] = segs_nns;
		cur_model.segment_voxels[i] = cloud_cur;
		cur_model.segment_normals[i] = normals_cur;

	}

/*
	if( debug_)
	{
    pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("cloud viewer segments");
    viewer->addPointCloud(cloud_deb,"segments");
		
		map<int,set<int> >::iterator it_v;
		for( it_v = cur_model.segment_nns.begin(); it_v != cur_model.segment_nns.end(); it_v++ )
    {
			int ref_id = (*it_v).first;
			Eigen::Vector4f ref_vec;
			pcl::compute3DCentroid(*cur_model.segment_voxels[(ref_id)],ref_vec);
      PointType p_ref;
      p_ref.x = ref_vec(0); p_ref.y = ref_vec(1);	p_ref.z = ref_vec(2);
			set<int>::iterator it_n;
      set<int> nns = (*it_v).second;
			int count = 0;
			for( it_n = nns.begin(); it_n != nns.end(); it_n++ )
      {
        int cur_id = (*it_n);
				//if( cur_model.segment_voxels.find(cur_id) == cur_model.segment_voxels.end() )
				//	continue;
				Eigen::Vector4f cur_vec;
				pcl::compute3DCentroid(*cur_model.segment_voxels[cur_id],cur_vec);				
        PointType p_cur;
        p_cur.x = cur_vec(0); p_cur.y = cur_vec(1); p_cur.z = cur_vec(2);
        stringstream ss;
        ss << "line" << ref_id << cur_id << count;
				count++;
        viewer->addLine(p_ref,p_cur,255,255,255,ss.str());
      }
    }
    while(!viewer->wasStopped())
    {
      viewer->spinOnce (100);
    }
    delete viewer;
	}*/
}


void vision::cluster_surface_patch(model &cur_model)
{
  map<int,set<int> >::iterator it;
  int count = 0; 
  map<int,int> seg_map;
  for( it = cur_model.segments.begin(); it != cur_model.segments.end(); it++ )
  {
    seg_map[(*it).first] = count;
    cur_model.surfaces[count].insert((*it).first);
    cur_model.surfaces_voxels[count] = cur_model.segment_voxels[(*it).first];
    cur_model.surfaces_normals[count] = cur_model.segment_normals[(*it).first];
    count++;
  }

	cout << "to assign neighbors" << endl;
  for( it = cur_model.surfaces.begin(); it != cur_model.surfaces.end(); it++ )
  { 
    set<int>::iterator its;
    for( its = cur_model.surfaces[(*it).first].begin(); its != cur_model.surfaces[(*it).first].end(); its++ )
    {
			if( cur_model.segment_nns.find(*its) == cur_model.segment_nns.end() )
			{
				set<int> nns;
				cur_model.segment_nns[*its] = nns;
				continue;
			}
      set<int> nns = cur_model.segment_nns[(*its)];
      set<int>::iterator its_2;
      for( its_2 = nns.begin(); its_2 != nns.end(); its_2++ )
        cur_model.surfaces_nns[(*it).first].insert(seg_map[(*its_2)]);
    } 
  }

  map<int,CloudPtr>::iterator it_v;
  ColorCloudPtr cloud_deb_surfaces(new ColorCloud);
  for( it_v = cur_model.surfaces_voxels.begin(); it_v != cur_model.surfaces_voxels.end(); it_v++ )
  {
    double r = rand() % 255; double g = rand() % 255; double b = rand() % 255;
    for( size_t i = 0; i < (*it_v).second->points.size(); i++ )
    {
      ColorPointType p;
      p.x = (*it_v).second->points[i].x;
      p.y = (*it_v).second->points[i].y;
      p.z = (*it_v).second->points[i].z;
      p.r = r; p.g = g; p.b = b;
      cloud_deb_surfaces->points.push_back(p);
    }
  }

  if( debug_ )
  {
    pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("cloud viewer Flat patches");
    viewer->addPointCloud(cloud_deb_surfaces,"surface");
 //   viewer->addPointCloud(cur_model.cloud,"original");
    while(!viewer->wasStopped())
      viewer->spinOnce (100);
    
  }
	cout << "surfaces done" << endl;
}


vector<string> vision::get_all_files(string dir_name,string pattern)
{
  vector<string> files;
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (dir_name.c_str())) != NULL) 
  {
    while ((ent = readdir (dir)) != NULL) {
      string name = ent->d_name;
      if( name.find(pattern) == string::npos )
        continue;
//      cout << name << endl;
      string file_name = dir_name + name;
      files.push_back(file_name);
    }
    closedir (dir);
  } 
  else
     cout << "can't open dir" << endl;

  return files;
}

vector<string> vision::sample_data(vector<string> files,double percentage)
{
	// ** get only 50 views per height for washington ** //
	vector<vector<string> > files_map(3);
	for( size_t i = 0; i < files.size(); i++ )
	{	
		string f_name = files[i];
		//cout << "orig: "<< f_name << endl;
		f_name = f_name.substr(0,f_name.find_last_of("_"));
		//cout << "p: " << f_name << endl;
		f_name = f_name.substr(f_name.find_last_of("_")+1);
		//cout << f_name << endl;
		if( f_name == "1" )
			files_map[0].push_back(files[i]);
    else if( f_name == "2" )
      files_map[1].push_back(files[i]);
    else if( f_name == "4" )
      files_map[2].push_back(files[i]);		
	}
	//cout << "height div done" << endl;
	vector<string> files_sample;
	for( size_t i = 0; i < files_map.size(); i++ )	
	{
		cout << files_map[i].size() << endl;
		set<int> seen_ids;
		for( size_t j = 0; j < 50; j++ )
		{
			int id;
			do{
				id = rand() % files_map[i].size();
			}while(seen_ids.find(id) != seen_ids.end());
			seen_ids.insert(id);
			files_sample.push_back(files_map[i][id]);
		}
	}
	//cout << "sampling done" << endl;
	files = files_sample;
	// ** // 

	int samp_size = files.size() * percentage;
  //int samp_size = files.size() * (percentage/2.);
  set<int> seen_ids;
  for( size_t i = 0; i < samp_size; i++ )
  {
    int id;
    do{
     id = rand() % files.size();
    }while(seen_ids.find(id) != seen_ids.end());
    seen_ids.insert(id);
  }
  vector<string> samp_files;
  set<int>::iterator it;
  vector<string> eval_files;
  for( it = seen_ids.begin(); it != seen_ids.end(); it++ )
  {
    int id = (*it);
    samp_files.push_back(files[id]);
  }
  vector<string> train_files;
  for( size_t i = 0; i < files.size(); i++ )
  {
    if( seen_ids.find(i) != seen_ids.end() )
      train_files.push_back(files[i]);
    else
      eval_files.push_back(files[i]);
  }
  cout << "number of sample files: " << samp_files.size() << endl;

  stringstream ss;
	ss << default_path_ << "evals-pcds/";
  string dir_eval_name = ss.str();
  boost::filesystem::path dir_eval(dir_eval_name);
  boost::filesystem::create_directories(dir_eval);

  for( size_t i = 0; i < eval_files.size(); i++ )
  {
    cout << "eval " << i << " : "<< eval_files.size() << endl;
    pcl::PCDReader reader;
    CloudPtr cloud (new Cloud);
    reader.read (eval_files[i], *cloud);

    string root_name = eval_files[i].substr(eval_files[i].find_last_of("/")+1);
    stringstream ss;
    cloud->width = 1;
    cloud->height = cloud->points.size();
    stringstream ssf;
    //ssf << dir_eval_name << "file" << i << ".pcd";
    ssf << dir_eval_name << root_name;
    pcl::io::savePCDFileASCII(ssf.str(),*cloud);
  }

  stringstream sst;
	sst << default_path_ << "train-pcds/";
  string dir_train_name = sst.str();
  boost::filesystem::path dir_train(dir_train_name);
  boost::filesystem::create_directories(dir_train);
  //int train_samp_size = files.size() * percentage;
  int train_samp_size = train_files.size() * percentage;
  set<int> train_seen;
  //for( size_t i = 0; i < train_files.size(); i++ )
  for( size_t i = 0; i < train_samp_size; i++ )
  {
    int id;
    do{
      id = rand() % train_files.size();
    }while(train_seen.find(id) != train_seen.end());
    train_seen.insert(id);
    //id = i;
    cout << "train " << i << " : " << train_samp_size << endl;
    pcl::PCDReader reader;
    CloudPtr cloud (new Cloud);
    reader.read (train_files[id], *cloud);

    string root_name = train_files[id].substr(train_files[id].find_last_of("/")+1);
    stringstream ss;
    cloud->width = 1;
    cloud->height = cloud->points.size();
    stringstream ssf;
    ssf << dir_train_name << root_name;
    pcl::io::savePCDFileASCII(ssf.str(),*cloud);
  }

  stringstream ssp;
  //ssp << "/media/TOSHIBA\ EXT/model-parts-pcds/";
	ssp << default_path_ <<	"model-parts-pcds/";
  string dir_pro_name = ssp.str();
  boost::filesystem::path dir_pro(dir_pro_name);
  boost::filesystem::create_directories(dir_pro);

  //int pro_size = train_files.size() * ( 1.0 - percentage) - 1;
	int pro_size = train_files.size() - train_samp_size;

  set<int> pro_seen;
  for( size_t i = 0; i < pro_size; i++ )
  {
    int id;
    do{
      id = rand() % train_files.size();
    }while((train_seen.find(id)!=train_seen.end()) || (pro_seen.find(id)!=pro_seen.end()) );
    //}while(pro_seen.find(id)!=pro_seen.end());
    pro_seen.insert(id);
    cout << "prototype  " << i << " : " << pro_size << endl;
    pcl::PCDReader reader;
    CloudPtr cloud (new Cloud);
    reader.read (train_files[id], *cloud);

    string root_name = train_files[id].substr(train_files[id].find_last_of("/")+1);
    stringstream ss;
    cloud->width = 1;
    cloud->height = cloud->points.size();
    stringstream ssf;
    ssf << dir_pro_name << root_name;
    pcl::io::savePCDFileASCII(ssf.str(),*cloud);
  }

  return samp_files;
}

model vision::decompose_object(CloudPtr cloud)
{
  dims_meter = dims_meter_;
  cout << "to estimate normals for decomposing" << endl;
  cloud_ = cloud;
  PointCloud<pcl::Normal>::Ptr normal_cloud = estimate_normal();
  
  model m;
  m.cloud = cloud;
  m.id = file_name_;
  m.cloud_normal = normal_cloud;
  
  segment_supervoxel(m);
  models_.push_back(m);
	cout << "decomposition done" << endl;
  return m; 
}


pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr vision::estimate_curvature(PointCloud<pcl::Normal>::Ptr cloud_normal,CloudPtr cloud)
{
  int k = 5;
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cvts(new pcl::PointCloud<pcl::PrincipalCurvatures>());
  pcl::PrincipalCurvaturesEstimation<PointType,pcl::Normal,pcl::PrincipalCurvatures> cvt;
  cvt.setInputCloud(cloud);
  cvt.setInputNormals(cloud_normal);
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
  cvt.setSearchMethod(tree);
  cvt.setSearchSurface(cloud);
  cvt.setKSearch(k);
  cvt.compute (*cvts);
  return cvts; 
}

void vision::extract_feature(model &m)
{
  map<int,CloudPtr>::iterator it;
  for( it = m.surfaces_voxels.begin(); it != m.surfaces_voxels.end(); it++ )
  {
    int i = (*it).first;
    int k = 5;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cvts(new pcl::PointCloud<pcl::PrincipalCurvatures>());
    pcl::PrincipalCurvaturesEstimation<PointType,pcl::Normal,pcl::PrincipalCurvatures> cvt;
    CloudPtr cloud = m.surfaces_voxels[i];
    PointCloud<pcl::Normal>::Ptr cloud_normal = m.surfaces_normals[i];
    cvt.setInputCloud(cloud);
    cvt.setInputNormals(cloud_normal);
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    cvt.setSearchMethod(tree);
    cvt.setSearchSurface(cloud);
    cvt.setKSearch(k);
    cvt.compute (*cvts);
    m.surface_cvts[i] = cvts;
    set<int>::iterator its;
    for( its = m.surfaces[i].begin(); its != m.surfaces[i].end(); its++ ) 
    {
      int id = (*its);
      CloudPtr cloud_p = m.segment_voxels[id];
      pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cvts_p(new pcl::PointCloud<pcl::PrincipalCurvatures>());
      pcl::PrincipalCurvaturesEstimation<PointType,pcl::Normal,pcl::PrincipalCurvatures> cvt_p;
      PointCloud<pcl::Normal>::Ptr cloud_normal_p = m.segment_normals[id];
      cvt.setInputCloud(cloud_p);
      cvt.setInputNormals(cloud_normal_p);
      cvt.setSearchMethod(tree);
      cvt.setSearchSurface(cloud_p);
      cvt.setKSearch(k);
      cvt.compute (*cvts_p);
      m.surface_voxel_cvts[i].push_back(cvts_p);
      m.surface_voxel_map[i].push_back(id);
      m.surface_segment_pts[i].push_back(cloud_p);
      m.surface_segment_map[i][(*its)] = m.surface_voxel_cvts[i].size() - 1;
    }
  }

}

void vision::extract_feature_axis(model &m)
{
  map<int,int> patch_surface_map;
  map<int,int> patch_surface_local_map;
  map<int,CloudPtr>::iterator it;
  int min_cloud_size = 10;
  for( it = m.surfaces_voxels.begin(); it != m.surfaces_voxels.end(); it++ )
  { 
    vector<int> sv_map = m.surface_voxel_map[(*it).first];
    for( size_t iv = 0; iv < sv_map.size(); iv++ )
    {
      parts_axis_call_++;
      patch_surface_map[sv_map[iv]] = (*it).first;
      patch_surface_local_map[sv_map[iv]] = iv;
      CloudPtr cloud = m.segment_voxels[sv_map[iv]];
      PointCloud<pcl::Normal>::Ptr cloud_normal = m.segment_normals[sv_map[iv]];

      if( parts_axis_call_ == 1 ){
        init_axis();
      }
      vector<vector<PointType> > axis_list = get_axis(cloud,cloud_normal);   
      Mat<double> trans_mat(4,4);
      trans_mat.fill(0);
      Mat<double> trans_mat_max(3,1);
      vec vec_max;
      Mat<double> center_mat(3,1);

      for( size_t i = 0; i < axis_list.size(); i++ )
      {
        Mat<double> tmp_mat(3,1);
      
        vec vec_tmp(3);
        tmp_mat(0,0) = axis_list[i][1].x - axis_list[i][2].x;
        tmp_mat(1,0) = axis_list[i][1].y - axis_list[i][2].y;
        tmp_mat(2,0) = axis_list[i][1].z - axis_list[i][2].z;
       
        vec_tmp(0) = tmp_mat(0,0); vec_tmp(1) = tmp_mat(1,0); vec_tmp(2) = tmp_mat(2,0);
        double n = norm(vec_tmp,2);
        vec_tmp /= n;
        tmp_mat = tmp_mat / n;
        trans_mat.submat(0,i,2,i) = tmp_mat;
        if( i == (int) ind_max ){
          trans_mat_max = tmp_mat;
          vec_max = vec_tmp;
          center_mat(0) = axis_list[i][2].x; center_mat(1) = axis_list[i][2].y; center_mat(2) = axis_list[i][2].z;
        }
      }
      trans_mat(0,3) = axis_list[0][1].x;
      trans_mat(1,3) = axis_list[0][1].y;
      trans_mat(2,3) = axis_list[0][1].z;
      trans_mat(3,3) = 1;
      m.axis_map[sv_map[iv]] = axis_list;
      m.trans_map[(*it).first].push_back(trans_mat_max);
      m.center_map[(*it).first].push_back(center_mat);
    }
  }
  for( it = m.surfaces_voxels.begin(); it != m.surfaces_voxels.end(); it++ )
  {
    vector<int> sv_map = m.surface_voxel_map[(*it).first];
    for( size_t i = 0; i < sv_map.size(); i++ )
    {
      Mat<double> trans_mat_ref = m.trans_map[(*it).first][i];  
      Mat<double> center_mat_ref = m.center_map[(*it).first][i];  
      set<int>::iterator its;
      Cube<double> desc(Bin_Angle,1,1);
      desc.fill(0);
      vector<vector<CloudPtr> > cloud_bins(Bin_Angle);
      vector<vector<PointCloud<pcl::Normal>::Ptr> > cloud_nn_bins(Bin_Angle);
      for( its = m.segment_nns[(*it).first].begin(); its != m.segment_nns[(*it).first].end(); its++ )
      {
        int s_id = patch_surface_map[*its];
        int l_id = patch_surface_local_map[*its];
        Mat<double> trans_mat_cur = m.trans_map[s_id][l_id];
        Mat<double> center_mat_cur = m.center_map[s_id][l_id];
        Mat<double> trans_diff = center_mat_cur - center_mat_ref;
        double theta = acos(dot(trans_mat_ref,trans_diff) / (norm(trans_mat_ref,2)*norm(trans_diff,2)) );
        int bin = 0 + ( Bin_Angle - 1 ) * ( (theta - 0) / M_PI);
        cloud_bins[bin].push_back(m.segment_voxels[*its]);
        cloud_nn_bins[bin].push_back(m.segment_normals[*its]); 
      }
      for( size_t j = 0; j < Bin_Angle; j++ )
      {
        if( cloud_bins[j].size() == 0 )
          continue;	
        double cvt = 0;
				CloudPtr cloud_boundary(new Cloud);
				vector<int> indices;
				for( size_t k = 0; k < cloud_bins[j].size(); k++ )
				{
					for( size_t ip = 0; ip < cloud_bins[j][k]->points.size(); ip++ )
            cloud_boundary->points.push_back(cloud_bins[j][k]->points[ip]); 
				}
        for( size_t ip = 0; ip < m.segment_voxels[sv_map[i]]->points.size(); ip++ )
        	cloud_boundary->points.push_back(m.segment_voxels[sv_map[i]]->points[ip]);
				for( size_t ip = 0;ip < cloud_boundary->points.size(); ip++)
					indices.push_back(ip);

        NormalEstimation<PointType, pcl::Normal> ne;
        ne.setInputCloud(cloud_boundary);
        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
        ne.setSearchMethod(tree);
        PointCloud<pcl::Normal>::Ptr cloud_normals (new PointCloud<pcl::Normal>);
        if( dims_meter_ )
          ne.setRadiusSearch (5.0);
        else
        	ne.setRadiusSearch (0.05);
        ne.setSearchSurface(cloud_boundary);
        float nx,ny,nz,cvt_;
        ne.computePointNormal(*cloud_boundary,indices,nx,ny,nz,cvt_);
        cvt = cvt_;
				desc(j,0,0) = cvt; 
      }
      m.surface_patch_axis[(*it).first].push_back(desc);
    }
  }
}


void vision::extract_feature_patch(model &m)
{
  double epsilon = 1e-1;
  map<int,CloudPtr>::iterator it;
  for( it = m.surfaces_voxels.begin(); it != m.surfaces_voxels.end(); it++ )
  {
    vector<int> sv_map = m.surface_voxel_map[(*it).first];
    for( size_t i = 0; i < sv_map.size(); i++ )
    {
      int id = sv_map[i];
      Cube<double> cube_patch(Bin_Angle,1,1);
      cube_patch.fill(0);
      pcl::Normal ref_normal = m.segment_normal[id];
      Eigen::Vector3f ref_vec;
      ref_vec(0) = ref_normal.normal_x; ref_vec(1) = ref_normal.normal_y; ref_vec(2) = ref_normal.normal_z;
      set<int>::iterator its;
      for( its = m.segment_nns[id].begin(); its != m.segment_nns[id].end(); its++ )
      {
        pcl::Normal cur_normal = m.segment_normal[(*its)];
        Eigen::Vector3f cur_vec;
        cur_vec(0) = cur_normal.normal_x; cur_vec(1) = cur_normal.normal_y; cur_vec(2) = cur_normal.normal_z;
        double val = ( cur_vec.dot(ref_vec) ) / ( cur_vec.norm() * ref_vec.norm() ) ;
        double theta = acos(val);
        int bin_val = (double) (Bin_Angle-1) * ( theta / M_PI ); 
        if( ( bin_val < 0 ) || ( bin_val > Bin_Angle ) )
          continue;
        cube_patch(bin_val,0,0)++;
      }
      double sum_patch = accu(cube_patch);
      if( sum_patch > epsilon )
        cube_patch /= sum_patch;
      m.surface_patch_normal[(*it).first].push_back(cube_patch);
    }
  }
  
}

void vision::save_feature(model m)
{
//  string dir_patch_name = "/media/TOSHIBA\ EXT/parts/";
	string dir_patch_name = default_path_ + "parts/";
  boost::filesystem::path dir_patch(dir_patch_name);
  boost::filesystem::create_directories(dir_patch);

  //string dir_subpatch_name = "/media/TOSHIBA\ EXT/sub-parts/";
	string dir_subpatch_name = default_path_ + "sub-parts/";
  boost::filesystem::path dir_subpatch(dir_subpatch_name);
  boost::filesystem::create_directories(dir_subpatch);


 // string dir_fet_name = "/media/TOSHIBA\ EXT/features/";
	string dir_fet_name = default_path_ + "features/";
  boost::filesystem::path dir_fet(dir_fet_name);
  boost::filesystem::create_directories(dir_fet);
  
  map<int,CloudPtr>::iterator it;

  for( it = m.surfaces_voxels.begin(); it != m.surfaces_voxels.end(); it++ )
  {
    pcl::PointCloud<PointNormal>::Ptr cloud_ap (new pcl::PointCloud<PointNormal>);
    CloudPtr cloud_p = (*it).second;
    PointCloud<pcl::Normal>::Ptr cloud_np = m.surfaces_normals[(*it).first];
    PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_cvp = m.surface_cvts[(*it).first];
    for( size_t i = 0; i < cloud_p->points.size(); i++ )
    {
      PointNormal point;
      point.x = cloud_p->points[i].x; point.y = cloud_p->points[i].y; point.z = cloud_p->points[i].z;
      point.normal_x = cloud_np->points[i].normal_x; point.normal_y = cloud_np->points[i].normal_y; point.normal_z = cloud_np->points[i].normal_z;
      cloud_ap->points.push_back(point);
    }
    if( ( cloud_ap->points.size() == 0 ) || ( cloud_cvp->points.size() == 0 ) )
      continue;
    stringstream ss;
    ss << dir_patch_name << file_name_ << "_part" << "_" << (*it).first << ".pcd";
    cloud_ap->width = 1;
    cloud_ap->height = cloud_ap->points.size();
    pcl::io::savePCDFileASCII (ss.str(),*cloud_ap);
    cout << "to save curvature" << endl;
    stringstream ssv;
    cloud_cvp->width = 1;
    cloud_cvp->height = cloud_cvp->points.size();
    ssv << dir_fet_name << file_name_ << "_cvt" << "_" << (*it).first << ".pcd";
    pcl::io::savePCDFileASCII (ssv.str(),*cloud_cvp);

    vector<pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr> voxel_cvts = m.surface_voxel_cvts[(*it).first];
    for( size_t i = 0; i < voxel_cvts.size(); i++ )
    {
      PointCloud<pcl::PrincipalCurvatures>::Ptr cvt_voxel = voxel_cvts[i];
      if( cvt_voxel->points.size() == 0 )
        continue;
      cvt_voxel->width = 1;
      cvt_voxel->height = cvt_voxel->points.size();
      stringstream ss_v;
      ss_v << dir_fet_name << file_name_ << "_cvt_voxel" << (*it).first << "_" << i << ".pcd";
      pcl::io::savePCDFileASCII (ss_v.str(),*cvt_voxel);

      CloudPtr cloud_patch(new Cloud);
      Cube<double> cube_patch = m.surface_patch_normal[(*it).first][i];
      for( int ip = 0; ip < cube_patch.n_elem; ip++ )
      { 
        PointType p;
        p.x = cube_patch(ip); p.y = 0; p.z = 0;
        cloud_patch->points.push_back(p);
      }
      if( cloud_patch->points.size() == 0 )
        continue;
      cloud_patch->width = 1; 
      cloud_patch->height = cloud_patch->points.size();
      stringstream ss_p;
      ss_p << dir_fet_name << file_name_  << "_normal_patch" << (*it).first << "_" << i << ".pcd";
      pcl::io::savePCDFileASCII (ss_p.str(),*cloud_patch);
    
      CloudPtr cloud_axis(new Cloud);
      Cube<double> cube_axis = m.surface_patch_axis[(*it).first][i];
      for( int ip = 0; ip < cube_axis.n_elem; ip++ )
      {
        PointType p;
        p.x = cube_axis(ip); p.y = 0; p.z = 0;
        cloud_axis->points.push_back(p);
      }
      if( cloud_axis->points.size() == 0 )
        continue;
      cloud_axis->width = 1;
      cloud_axis->height = cloud_axis->points.size();
      stringstream ss_ax;
      ss_ax << dir_fet_name << file_name_ << "_axis_patch" << (*it).first << "_" << i << ".pcd";
      pcl::io::savePCDFileASCII (ss_ax.str(),*cloud_axis);      

      CloudPtr cloud_voxel = m.segment_voxels[m.surface_voxel_map[(*it).first][i]];
      if( cloud_voxel->points.size() == 0 )
        continue;

      cloud_voxel->width = 1; cloud_voxel->height = cloud_voxel->points.size();
      stringstream ss_vo;
      ss_vo << dir_subpatch_name << file_name_  << "_patch" << (*it).first << "_" << i << ".pcd";
      pcl::io::savePCDFileASCII (ss_vo.str(),*cloud_voxel);
    }
  }

}


vector<Hcluster> vision::read_clusters_patch(vector<string> paths,vector<model> &models_all)
{
  //vector<Hcluster> clusters;
  vector<Hcluster> clusters(paths.size());
  for( size_t i = 0; i < paths.size(); i++ )
  {
    Hcluster cluster;
//    cout << paths[i] << endl;
    string id = paths[i].substr(paths[i].find_last_of("cls") + 1);
    int c_id;
    stringstream ss(id);
    ss >> c_id;
 //  cout << "cls id: " << id << endl;

    paths[i] = paths[i] + "/";
    cluster_name_map_[i] = paths[i]; 

    vector<model> models = read_prototypes(paths[i],false);
 //   cout << models.size() << endl;
    for( size_t j = 0; j < models.size(); j++ )
    {
      cluster.mems_pts.push_back(models[j].cloud);
      map<int,vector<Cube<double> > >::iterator it;
      //for( it = models[j].surface_patch_normal.begin(); it != models[j].surface_patch_normal.end(); it++ )
      for( it = models[j].surface_patch_axis.begin(); it != models[j].surface_patch_axis.end(); it++ )
      {
        /*for( size_t ip = 0; ip < models[j].surface_patch_normal[(*it).first].size(); ip++ )
          cluster.memebers.push_back(models[j].surface_patch_normal[(*it).first][ip]);*/
        for( size_t ip = 0; ip < models[j].surface_patch_axis[(*it).first].size(); ip++ )
          cluster.memebers.push_back(models[j].surface_patch_axis[(*it).first][ip]);
      }
    }    
    clusters[c_id] = cluster;
  }
  return clusters;
}

vector<Hcluster> vision::read_clusters(vector<string> paths,vector<model> &models_all)
{
  vector<Hcluster> clusters;
  for( size_t i = 0; i < paths.size(); i++ )
  {
    cout << paths[i] << endl;
    paths[i] = paths[i] + "/";
    cluster_name_map_[i] = paths[i];
    vector<model> models = read_prototypes(paths[i],false);
    cout << models.size() << endl;
    Hcluster cluster;
    Cube<double> center(Hist_Bin+1,Hist_Bin+1,Hist_Bin+1);
    center.fill(0);
    Cube<double> center_n(Hist_Bin+1,Hist_Bin+1,Hist_Bin+1);
    center_n.fill(0);

    int n_elems = pow(Hist_Bin+1,3);
    Mat<double> cov(n_elems,n_elems);
    cov.fill(0);
    Mat<double> cov_n(n_elems,n_elems);
    cov_n.fill(0);
    for( size_t j = 0; j < models.size(); j++ )
    {
      cluster.memebers.push_back(models[j].surface_hist[0]);
      hists.push_back(models[j].surface_hist[0]);
      //hists_border.push_back(models[j].surface_border_hist[0]); 
    //  cluster.memebers_n.push_back(models[j].surface_border_hist[0]);
      cluster.members_ids.push_back(hists.size()-1);
      models_all.push_back(models[j]);
      model_map[hists.size()-1] = models_all.size()-1;

      center += models[j].surface_hist[0];
     // center_n += models[j].surface_border_hist[0];
    }
    center /= models.size();
    center_n /= models.size();
    cout << "to create cov matrix" << endl;
    for( size_t j = 0; j < models.size(); j++ )
    {
      Cube<double> diff_cube = models[j].surface_hist[0] - center;
      //Cube<double> diff_cube_n = models[j].surface_border_hist[0] - center_n;
      Mat<double> diff_cur(n_elems,1);
      Mat<double> diff_cur_n(n_elems,1);
      for( size_t ie = 0; ie < diff_cube.n_elem; ie++ ){
        diff_cur(ie,0) += diff_cube[ie];
   //     diff_cur_n(ie,0) += diff_cube_n[ie];
      }
      Mat<double> dist = diff_cur * diff_cur.t();
      Mat<double> dist_n = diff_cur_n * diff_cur_n.t();
      cov += dist;
     // cov_n += dist_n;
    }
    cov /= models.size();
    cov_n /= models.size();
    cov *= eye(cov.n_rows,cov.n_cols);
    //cov_n *= eye(cov_n.n_rows,cov_n.n_cols);
    double epsilon = 0.001;
    for( size_t i = 0; i < cov.n_rows; i++ )
    {
      if( fabs(cov(i,i)) < epsilon )
        cov[i] = epsilon;
      if( fabs(cov_n(i,i)) < epsilon )
        cov_n[i] = epsilon;   
    }
 
    cluster.center = center;
    //cluster.center_n = center_n;
    cluster.cov_mat = cov;
    //cluster.cov_mat_n = cov_n;

    clusters.push_back(cluster);
  }
  return clusters;
}

Cube<double> vision::histogram_from_curvature(pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cvts)
{
  max_cvt_y = FLT_MIN;  max_cvt_x = FLT_MIN; max_cvt_z = FLT_MIN;
  min_cvt_y = FLT_MAX;  min_cvt_x = FLT_MAX; min_cvt_z = FLT_MAX;

  for( size_t i = 0; i < cvts->points.size(); i++ )
  {
    if( ( isnan(fabs(cvts->points[i].principal_curvature_y)) ) || ( isinf(fabs(cvts->points[i].principal_curvature_y)) ) ||
      ( isnan(fabs(cvts->points[i].principal_curvature_x)) ) || ( isinf(fabs(cvts->points[i].principal_curvature_x)) ) ||
      ( isnan(fabs(cvts->points[i].principal_curvature_z)) ) || ( isinf(fabs(cvts->points[i].principal_curvature_z)) ) )
        continue;
 
    if( cvts->points[i].principal_curvature_y < min_cvt_y ) 
      min_cvt_y = cvts->points[i].principal_curvature_y;
    if( cvts->points[i].principal_curvature_x < min_cvt_x )
      min_cvt_x = cvts->points[i].principal_curvature_x;
    if( cvts->points[i].principal_curvature_z < min_cvt_z )
      min_cvt_z = cvts->points[i].principal_curvature_z;
    if( cvts->points[i].principal_curvature_y > max_cvt_y )
      max_cvt_y = cvts->points[i].principal_curvature_y;
    if( cvts->points[i].principal_curvature_x > max_cvt_x )
      max_cvt_x = cvts->points[i].principal_curvature_y;
    if( cvts->points[i].principal_curvature_z > max_cvt_z )
      max_cvt_z = cvts->points[i].principal_curvature_z;
  }
 
  Cube<double> hist(Hist_Bin+1,Hist_Bin+1,Hist_Bin+1);
  hist.zeros();
  for( size_t ip = 0; ip < cvts->points.size(); ip++ )
  {
    int bin_y = ( (double)Hist_Bin / ( max_cvt_y - min_cvt_y ) ) * ( cvts->points[ip].principal_curvature_y - min_cvt_y );
    int bin_z = ( (double)Hist_Bin / ( max_cvt_z - min_cvt_z ) ) * ( cvts->points[ip].principal_curvature_z - min_cvt_z );
    int bin_x = ( (double)Hist_Bin / ( max_cvt_x - min_cvt_x ) ) * ( cvts->points[ip].principal_curvature_x - min_cvt_x );
    if( ( bin_x >= 0 ) && ( bin_x < Hist_Bin+1 ) && ( bin_y >= 0 ) && ( bin_y < Hist_Bin+1 ) && ( bin_z >= 0 ) && ( bin_z < Hist_Bin+1 ) )
      hist(bin_x,bin_y,bin_z) ++;
  }
  normalize_hist(hist);
  return hist; 
}

model vision::read_model(vector<string> files,bool is_prototype)
{
  model m;
	CloudPtr cloud_ref(new Cloud);
	m.cloud = cloud_ref;
  map<string,int> seg_map;
  map<string,int> surf_map;
  set<int> surf_set;
  map<int,set<int> > surf_segs_map;
  set<string> cues;
  string root;
  for( size_t i = 0; i < files.size(); i++ )
  {
   // cout << files[i] << endl;
    string root = files[i].substr(files[i].find_last_of("/")+1);
    root  = root.substr(0,root.find("."));
    string cue = "";
    if( root.find("_cvt_voxel") != string::npos )
    {
      cue = "_cvt_voxel";
      string cue_surf = root.substr(root.find(cue)+cue.length());
      string cue_seg = cue_surf.substr(cue_surf.find_last_of("_")+1);
      cue_surf = cue_surf.substr(0,cue_surf.find("_"));
      stringstream su(cue_seg);
      int cue_seg_n;
      su >> cue_seg_n;
      stringstream se(cue_surf);
      int cue_surf_n;
      se >> cue_surf_n;
      seg_map[files[i]] = cue_seg_n;
      surf_map[files[i]] = cue_surf_n;
      surf_set.insert(cue_surf_n);
      surf_segs_map[cue_surf_n].insert(cue_seg_n);
    }
    else if( root.find("_cvt") != string::npos )
    {
      cue = "_cvt";
      string cue_surf = root.substr(root.find(cue)+cue.length());
      cue_surf = cue_surf.substr(cue_surf.find("_")+1);
      cue_surf = cue_surf.substr(0,cue_surf.find_last_of("_"));

      stringstream su(cue_surf);
      int cue_surf_n;
      su >> cue_surf_n;
      surf_map[files[i]] = cue_surf_n;
      surf_set.insert(cue_surf_n);
    }
    else if( root.find("_part") != string::npos )
    {
      cue = "_part";
      string cue_surf = root.substr(root.find(cue)+cue.length());
      cue_surf = cue_surf.substr(cue_surf.find("_")+1);
      cue_surf = cue_surf.substr(0,cue_surf.find_last_of("_"));

      stringstream su(cue_surf);
      int cue_surf_n;
      su >> cue_surf_n;
      surf_map[files[i]] = cue_surf_n;
      surf_set.insert(cue_surf_n);
    }
    else if( root.find("_normal_patch") != string::npos )
    {
      cue = "_normal_patch";
      string cue_surf = root.substr(root.find(cue)+cue.length());
      string cue_seg = cue_surf.substr(cue_surf.find_last_of("_")+1);
      cue_surf = cue_surf.substr(0,cue_surf.find("_"));

      stringstream su(cue_seg);
      int cue_seg_n;
      su >> cue_seg_n;
      stringstream se(cue_surf);
      int cue_surf_n;
      se >> cue_surf_n;
      seg_map[files[i]] = cue_seg_n;
      surf_map[files[i]] = cue_surf_n;
      surf_set.insert(cue_surf_n);
      surf_segs_map[cue_surf_n].insert(cue_seg_n);
    }
    else if( root.find("_axis_patch") != string::npos )
    {
      cue = "_axis_patch";
      string cue_surf = root.substr(root.find(cue)+cue.length());
      string cue_seg = cue_surf.substr(cue_surf.find_last_of("_")+1);
      cue_surf = cue_surf.substr(0,cue_surf.find("_"));

      stringstream su(cue_seg);
      int cue_seg_n;
      su >> cue_seg_n;
      stringstream se(cue_surf);
      int cue_surf_n;
      se >> cue_surf_n;
      seg_map[files[i]] = cue_seg_n;
      surf_map[files[i]] = cue_surf_n;
      surf_set.insert(cue_surf_n);
      surf_segs_map[cue_surf_n].insert(cue_seg_n);
    }
    else if( root.find("_patch") != string::npos )
    {
      cue = "_patch";
			//cout << "patch 0: " << root << endl;
      string cue_surf = root.substr(root.find(cue)+cue.length());
			//string cue_surf = root.substr(root.find_first_of(cue)+1);
			//cout << "patch 1: " << cue_surf << endl;
      string cue_seg = cue_surf.substr(cue_surf.find_last_of("_")+1);
      cue_surf = cue_surf.substr(0,cue_surf.find("_"));

      stringstream su(cue_seg);
      int cue_seg_n;
      su >> cue_seg_n;
      stringstream se(cue_surf);
      int cue_surf_n;
      se >> cue_surf_n;
      seg_map[files[i]] = cue_seg_n;
      surf_map[files[i]] = cue_surf_n;
      surf_set.insert(cue_surf_n);
      surf_segs_map[cue_surf_n].insert(cue_seg_n);

    }
    if( ( cues.size() == 0 ) && ( cue.length() > 0 ) )
      root = files[i].substr(0,root.find(cue));    
    cues.insert(cue);
  }
   
  map<int,int> surf_map_n;
  vector<map<int,int> > seg_map_n;
  set<int>::iterator its;
  int count = 0;
  for( its = surf_set.begin(); its != surf_set.end(); its++ )
  {
    surf_map_n[(*its)] = count;
    int s = surf_segs_map[(*its)].size();
    if( cues.find("_cvt_voxel") != cues.end() ){
      m.surface_voxel_cvts[count].resize(s);
      m.surface_voxel_hist[count].resize(s);
    }
    int count_s = 0;
    set<int>::iterator its_2;
    for( its_2 = surf_segs_map[(*its)].begin(); its_2 != surf_segs_map[*its].end(); its_2++ ){
      map<int,int> map_tmp;
      map_tmp[*its_2] = count_s;  
      seg_map_n.push_back(map_tmp); 
      count_s++;
    }
    m.surface_patch_normal[count].resize(s); 
    m.surface_patch_axis[count].resize(s);
    count++;
  }

	//cout << "root is: " << root << endl;

  vector<string> cur_files = files;

    for( size_t i = 0; i < cur_files.size(); i++ )
    {
      //cout << "read model: " << cur_files[i] << endl;
      if( cur_files[i].find("cvt") != string::npos )
      {
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cvt (new pcl::PointCloud<pcl::PrincipalCurvatures>);
        reader.read (cur_files[i], *cvt);
        Cube<double> hist = histogram_from_curvature(cvt);

        if( cur_files[i].find("voxel") != string::npos )
        {
          int id = surf_map[cur_files[i]];
          id = surf_map_n[id];
          int sid = seg_map[cur_files[i]];
          map<int,int> tmp_map;
          tmp_map = seg_map_n[id];
          sid = tmp_map[sid];
          m.surface_voxel_cvts[id][sid] = cvt;
          m.surface_voxel_hist[id][sid] = hist;
        }
        else{
          int id = surf_map[cur_files[i]];
          id = surf_map_n[id];
          m.surface_cvts[id] = cvt;
          m.surface_hist[id] = hist;
        }
      }
      else if( cur_files[i].find("part") != string::npos )
      {
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read(cur_files[i], *cloud);
        int id = surf_map[cur_files[i]];
        id = surf_map_n[id];
        m.surfaces_voxels[id] = cloud;
        if( ! is_prototype )
          m.cloud = cloud;
      }
      else if( cur_files[i].find("normal_patch") != string::npos )
      {
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read(cur_files[i], *cloud);
        Cube<double> cube_patch(Bin_Angle,1,1);
        cube_patch.fill(0);
        for( size_t j = 0; j < cloud->points.size(); j++ )
          cube_patch(j) = cloud->points[j].x;
        int id = surf_map[cur_files[i]];
        id = surf_map_n[id];
        int sid = seg_map[cur_files[i]];
        map<int,int> tmp_map;
        tmp_map = seg_map_n[id];
        sid = tmp_map[sid];
        m.surface_patch_normal[id][sid] = cube_patch;
      }
      else if( cur_files[i].find("axis_patch") != string::npos )
      {
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read(cur_files[i], *cloud);
        Cube<double> cube_patch(Bin_Angle,1,1);
        cube_patch.fill(0);
        for( size_t j = 0; j < cloud->points.size(); j++ )
          cube_patch(j,0,0) = cloud->points[j].x;
        int id = surf_map[cur_files[i]];
        id = surf_map_n[id];
        int sid = seg_map[cur_files[i]];
        map<int,int> tmp_map;
        tmp_map = seg_map_n[id];
        sid = tmp_map[sid];
        m.surface_patch_axis[id][sid] = cube_patch;
      }
      /*else if( cur_files[i].find("patch") != string::npos )
      {
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read(cur_files[i], *cloud);
        int sid = seg_map[cur_files[i]];
        m.segment_voxels[sid] = cloud;
      }*/
      else if( cur_files[i].find("patch") != string::npos )
      {
				/*string temp = root + ".pcd";
				if( cur_files[i].find(temp) != string::npos )
				{
					cout << "cloud found" << endl;
					pcl::PCDReader reader;
          CloudPtr cloud (new Cloud);
          reader.read(cur_files[i], *cloud);
          m.cloud = cloud;					
				}
				else
				{*/
					pcl::PCDReader reader;
					CloudPtr cloud (new Cloud);
					reader.read(cur_files[i], *cloud);
					int sid = seg_map[cur_files[i]];
					m.segment_voxels[sid] = cloud;
				//}
      }			
      else
      {
        if( is_prototype ){
          pcl::PCDReader reader;
          CloudPtr cloud (new Cloud);
          reader.read(cur_files[i], *cloud);
          m.cloud = cloud;
        }
      }
    }
    m.id = root;
    if( is_prototype )
      model_prototypes_[root].push_back(m);
    

  return m; 
}

vector<model> vision::read_prototypes(string path,bool is_prototype)
{
  vector<model> models;
  vector<model> tmp_models; 
  vector<string> files = get_all_files(path);
  cout << "to read prototypes" <<endl;
  if( is_prototype )
    model_prototypes_.clear(); 
  map<string,vector<string> > file_map;
  for( size_t i = 0; i < files.size(); i++ )
  {
    cout << files[i] << endl;
    string root = files[i].substr(files[i].find_last_of("/")+1);
    root  = root.substr(0,root.find("."));
    string cue = "";
    if( root.find("_cvt_voxel") != string::npos )
      cue = "_cvt_voxel";
    else if( root.find("_cvt") != string::npos )
      cue = "_cvt";  
    else if( root.find("_part") != string::npos )
      cue = "_part";
    else if( root.find("_normal_patch") != string::npos )
      cue = "_normal_patch";
    else if( root.find("_axis_patch") != string::npos )
      cue = "_axis_patch";
    else if( root.find("_patch") != string::npos )
      cue = "_patch";
    if( cue.length() > 1 ) 
      root = root.substr(0,root.find(cue));
    file_map[root].push_back(files[i]);
  } 
  

  cout << "to read model" << endl;
  map<string,vector<string> >::iterator it;
  for( it = file_map.begin(); it != file_map.end(); it++ )
  {
    cout << "file: " << (*it).first << endl;
    vector<string> cur_files = (*it).second;
    model m;
    m = read_model(cur_files,is_prototype);
		m.id = ((*it).first);
		if( (m.cloud->points.size() == 0 ) && (is_prototype) ){	
			cout << "rejected is: " << m.id << endl;
			continue;
		}
		cout << "read is: " << m.id << endl;
    tmp_models.push_back(m);
  }
  
  models = tmp_models;  
  return models;
}


void vision::normalize_hist(Cube<double> &hist)
{
  double m = 0;
  for( int i = 0; i < hist.n_elem; i++ )
  {
    if( hist(i) > m )
      m = hist(i);
  }
  hist = hist / m;
}

void vision::normalize_hist_gaus(Cube<double> &hist)
{
  double mu = 0; 
  mu = accu(hist) / hist.n_elem;
  cout << "mean: " << mu << endl; 
  hist = hist - mu;

  double var = 0;
  for( int i = 0; i < hist.n_elem; i++ )
    var += pow(hist(i),2);

  var /= (hist.n_elem * (hist.n_elem - 1)); 
  var = sqrt(var);
  cout << "var: " << var << endl;
  hist = hist / var;
}

void vision::get_cluster_threshold()
{
  std::sort(pair_dists_.begin(),pair_dists_.end());
  for( size_t i = 0; i < pair_dists_.size(); i++ )
    cout << pair_dists_[i] << " ";
  cout << endl;
  if( pair_dists_.size() % 2 == 0 )
    threshold_ = pair_dists_[pair_dists_.size()/2];
  else
  {
    int d1 = pair_dists_.size()/2;
    int d2 = (pair_dists_.size()/2) + 1;
    threshold_ = (pair_dists_[d1] + pair_dists_[d2]) / 2.;
  }
}

double vision::agg_clustering(vector<model> models,vector<Hcluster> &clusters)
{
	cout << "agg to make initial matrix: " << hists.size() << endl;
  Mat<double> dist_mat(hists.size(),hists.size());
  dist_mat.fill(0);

	Mat<double> dists_;
	dists_.set_size(hists.size(),hists.size());
	dists_.fill(FLT_MAX);	

	cout << "after init" << endl;
	if( !object_ins_cls_ )
	{
  	dist_mat_ = dist_mat;
		cout << "after init dist_mat_ " << endl;
  	for( size_t i = 0; i < hists.size(); i++ )
  	{
    	for( size_t j = i+1; j < hists.size(); j++ )
    	{
      	double dist;
      	if( parts_cls_ )
        	dist = chi_hist_distance(hists[i],hists[j]);
      	else
        	dist = shape_context_distance(hists[i],hists[j]);

      	dist_mat_(i,j) = dist;
      	dist_mat_(j,i) = dist;


        dists_(i,j) = dist;
        dists_(j,i) = dist;								
    	}
    	Hcluster cluster;
    	cluster.memebers.push_back(hists[i]);
    	cluster.members_ids.push_back(i);
    	clusters.push_back(cluster);
  	}
	}
	//cout << "to save dist mat" << endl;
	//dist_mat_.save("dist-mat.mat", arma_ascii);
	//return 0;
  double dist_thresh = threshold_;
  bool changed = true;
  int count = 0;
	double epsilon = 1e-5;
  while( changed )
  {
    cout << "iter: " << count << endl;
    vector<Hcluster> cur_clusters;
    map<int,vector<int> > cls_map;
    set<int> seen_ids;


		map<int,set<int> > map_data;
		vec min_vals(clusters.size()); 
		for( int i = 0; i < clusters.size(); i++)
		{
			Mat<double> temp = dists_.row(i);
			vec v(temp.n_elem);
			for(int iv = 0; iv < temp.n_elem; iv++)
				v(iv) = temp.at(iv);
			uword index;
			double min_val = v.min(index);
			map_data[i].insert(index);
			min_vals[i] = min_val;
			uvec v_vals = find(v <= (min_val + epsilon) );
			if( v_vals.n_elem <= 1 )
				continue;	
			for( size_t j = 0; j < v_vals.n_elem; j++ )
				map_data[i].insert(v_vals(j));	
		}

		uvec min_indx = sort_index(min_vals);
		bool valid = true;
		for( int i = 0;( i < clusters.size() && (valid) ); i++)
		{
			int id = min_indx(i);	
			double val = min_vals[id];
			if( val > dist_thresh )
			{
				valid = false;
				continue;
			}
			set<int>::iterator its;
			if( seen_ids.find(id) != seen_ids.end() )
				continue;
			bool valid_local = true;
			for( its = map_data[id].begin(); its != map_data[id].end(); its++ )
			{
        if( seen_ids.find(*its) != seen_ids.end() )
        {
					valid_local = false;
					break;
				}				
			}
			if(!valid_local)
				continue;
			seen_ids.insert(id);
			for( its = map_data[id].begin(); its != map_data[id].end(); its++ ){
				seen_ids.insert(*its);
				cls_map[id].push_back(*its);
			}
		}


/*
	  uword row,col;
    double min_val = dists_.min(row,col);
		if( min_val <= dist_thresh ){
			cls_map[row].push_back(col);
			seen_ids.insert(row);
			seen_ids.insert(col);
		}
		
*/
//-------------------------------------------------------------------------------------------

	
/*		int id1,id2;
		bool found_min = false;
		double min_dist_all = FLT_MAX;
    for( size_t i = 0; i < clusters.size(); i++ )
    {
			if(seen_ids.find(i) != seen_ids.end() )
				continue;
			int id1,id2;
			bool found_min = false;
			
      for( size_t j = i+1; j < clusters.size(); j++ )
      {
				double cur_val = 0;
        if( agg_max_ )
          //cur_dist_mat(i,j) = dist_cluster_max(clusters[i],clusters[j]);
					cur_val = dist_cluster_max(clusters[i],cur_clusters[j]);
        else
         // cur_dist_mat(i,j) = dist_cluster_avg(clusters[i],clusters[j]);
					cur_val = dist_cluster_avg(clusters[i],cur_clusters[j]);
        //cur_dist_mat(j,i) = cur_dist_mat(i,j);
	
				if( ( cur_val <= dist_thresh ) && ( cur_val < min_dist_all ) )
				{
					//min_dist_all = cur_dist_mat(i,j);
					min_dist_all = cur_val;
					id1 = i; id2 = j;
					found_min = true;	
				}
      }

    }

			
		if( found_min )
		{
			cls_map[id1].push_back(id2);
			seen_ids.insert(id1);
			seen_ids.insert(id2);
		}
*/	
		//cout << "to update" << endl;
    map<int,vector<int> >::iterator itm;
    for( itm = cls_map.begin(); itm != cls_map.end(); itm++ )
    {
      int id = (*itm).first;
      Hcluster cluster = clusters[id];
      vector<int> mems = (*itm).second;
      for( size_t i = 0; i < mems.size(); i++ )
      {
        for( size_t j = 0; j < clusters[mems[i]].memebers.size(); j++ )
        {
          cluster.memebers.push_back(clusters[mems[i]].memebers[j]);
          cluster.members_ids.push_back(clusters[mems[i]].members_ids[j]);  
        }
      }
      cur_clusters.push_back(cluster);
    }

    for( size_t i = 0; i < clusters.size(); i++ )
    {
      if( seen_ids.find(i) == seen_ids.end() )
        cur_clusters.push_back(clusters[i]);
    }
		//cout << "to resize" << endl;
		dists_.resize(cur_clusters.size(),cur_clusters.size());
		dists_.fill(FLT_MAX);
		//cout << "to enter vals" << endl;
    for( size_t i = 0; i < cur_clusters.size(); i++ )
    {
      for( size_t j = i+1; j < cur_clusters.size(); j++ )
      {
        double cur_val = 0;
        if( agg_max_ )
          //cur_dist_mat(i,j) = dist_cluster_max(clusters[i],clusters[j]);
          cur_val = dist_cluster_max(cur_clusters[i],cur_clusters[j]);
        else
         // cur_dist_mat(i,j) = dist_cluster_avg(clusters[i],clusters[j]);
          cur_val = dist_cluster_avg(clusters[i],cur_clusters[j]);
        //cur_dist_mat(j,i) = cur_dist_mat(i,j);
				dists_(i,j) = cur_val;
				dists_(j,i) = cur_val;
      }

    }
		//cout << "iter done" << endl;
		cout << "#of cls: " << cur_clusters.size() << endl;
    changed = false;
    if( cls_map.size() >= 1 )
      changed = true;
    if( cur_clusters.size() >= 1 )
      clusters = cur_clusters;
    count++;  
  }

	cout << "after agg clustering" << endl;
  double comp = 0;
 
  cout << "in threshold: " << threshold_ << " ,cluster size: " << clusters.size() << endl;
  return comp;
}

double vision::dist_cluster(Hcluster cluster1,Hcluster cluster2,Mat<double> dist_mat)
{
  double dist = FLT_MAX;
  for( size_t i = 0; i < cluster1.members_ids.size(); i++ )
  {  
    int id1 = cluster1.members_ids[i];
    for( size_t j = 0; j < cluster2.members_ids.size(); j++ )
    {
      int id2 = cluster2.members_ids[j];
      if( dist_mat(id1,id2) < dist )
        dist = dist_mat(id1,id2);
    }  
  }
  return dist;
}

double vision::dist_cluster_max(Hcluster cluster1,Hcluster cluster2)
{
  double dist = FLT_MIN;
  for( size_t i = 0; i < cluster1.members_ids.size(); i++ )
  {
    int id1 = cluster1.members_ids[i];
    for( size_t j = 0; j < cluster2.members_ids.size(); j++ )
    {
      int id2 = cluster2.members_ids[j];
      if( dist_mat_(id1,id2) > dist )
        dist = dist_mat_(id1,id2);

  /*    int tid = table_size_ * (i/table_size_) + (j / table_size_);
      int i1 = i % table_size_;
      int j1 = j % table_size_;
      double cur_dist = dist_mats_[tid](i1,j1);

			if( cur_dist > dist )
				dist = cur_dist;*/

    }
  }
  return dist;
}

double vision::dist_cluster_avg(Hcluster cluster1,Cube<double> desc)
{
  double dist = 0;
  for( size_t i = 0; i < cluster1.memebers.size(); i++ )
  {
    Cube<double> fet_ref = cluster1.memebers[i];
    if( parts_cls_ )
      dist += chi_hist_distance(fet_ref,desc);
    else
     dist += shape_context_distance(fet_ref,desc);
  }
  dist /= cluster1.memebers.size();
  return dist;
}

double vision::dist_cluster_avg(Hcluster cluster1,Hcluster cluster2)
{
  double dist = 0;
  for( size_t i = 0; i < cluster1.members_ids.size(); i++ )
  {
    int id1 = cluster1.members_ids[i];
    for( size_t j = 0; j < cluster2.members_ids.size(); j++ )
    {
      int id2 = cluster2.members_ids[j];
      dist += dist_mat_(id1,id2);
/*
      int tid = table_size_ * (i/table_size_) + (j / table_size_);
      int i1 = i % table_size_;
      int j1 = j % table_size_;
      double cur_dist = dist_mats_[tid](i1,j1);
			dist += cur_dist;*/
    }
  }
  dist /= ( cluster1.members_ids.size() * cluster2.members_ids.size() );
  return dist;
}

vector<Hcluster> vision::cluster_features(vector<model> models)
{
  int count = 0;
  hists.clear();
  map<int,int> surface_map;
  map<int,int> voxel_map;
  if( parts_ )
  {
    for( size_t i = 0; i < models.size(); i++ )
    {
      map<int,Cube<double> >::iterator its;
      for( its = models[i].surface_hist.begin(); its != models[i].surface_hist.end(); its++ )
      { 
        hists.push_back((*its).second);
        model_map[count] = i;
        model_local_map[count] = (*its).first;
        hists_border.push_back(models[i].surface_border_hist[(*its).first]);
        count++;
      }
    }
  }
  else
  {
    for( size_t i = 0; i < models.size(); i++ )
    {
      map<int,CloudPtr>::iterator its; 
      model m = models[i];
      //cout << "model " << i << " : "<< m.surfaces_voxels.size() << endl;			
      for( its = m.surfaces_voxels.begin(); its != m.surfaces_voxels.end(); its++ )
      {
        int id = (*its).first;
			//	cout << "#of axis: " << m.surface_patch_axis[id].size() << endl;
        for( size_t ip = 0; ip < m.surface_patch_axis[id].size(); ip++ )
        {
          hists.push_back(m.surface_patch_axis[id][ip]);
          model_map[count] = i;
					if( m.surface_voxel_map.find(id) != m.surface_voxel_map.end() ) 
						model_local_map[count] = m.surface_voxel_map[id][ip];
					else
						model_local_map[count] = id;
          surface_map[count] = id;
          voxel_map[count] = ip;
          count++;
        }
      }
    }
  }

	cout << "#of hists: " << hists.size() << endl;
  double best_thresh = FLT_MAX;
  vector<Hcluster> clusters;
  std::sort(pair_dists_.begin(),pair_dists_.end());
  double best_comp = FLT_MAX;
  int mid_id = pair_dists_.size()/2;
  //int min_id = mid_id / 2;
  int min_id = 0;
 // int max_id = mid_id + (pair_dists_.size() - pair_dists_.size()/2) / 2; 
  int max_id = mid_id +1;
    vector<Hcluster> clusters_cur;
  cout << "pair dists size: " << pair_dists_patch_.size() << endl;
  if( agg_thresh_min_ )
    threshold_ = pair_dists_patch_[0];
  else
  {
    int id = pair_dists_patch_.size() /2;
    threshold_ = pair_dists_patch_[id];
  }

  ofstream ofs;
	stringstream ss;
	ss << default_path_ << "cls-patch-thresh.txt";
	string path_name = ss.str();
  //ofs.open("/home/c7031086/Documents/codes/code-comp/build/cls-patch-thresh.txt");
	ofs.open(path_name.c_str());
  ofs << threshold_ << endl;
  ofs.close();

  cout << "clustering threshold: "<< threshold_ << endl;
  parts_cls_ = false;
  double comp = agg_clustering(models,clusters_cur);
  if( comp < best_comp )
  {
    best_comp = comp;
    best_thresh = threshold_;
    clusters = clusters_cur;
  }
  cout << "to make codebook" << endl;
  make_code_book(clusters);
  cout << "clusters num " << clusters.size() << endl;
  cout << "compactness: " << best_comp << endl;
  cout << "best threshold: " << best_thresh << endl;
  for( size_t i = 0; i < clusters.size(); i++ )
  {
    stringstream ssf;
    //ssf << "/media/TOSHIBA\ EXT/clusters/" << "cls" << i << "/";
		ssf << default_path_ <<	"clusters/" << "cls" << i << "/";
    string dir_cls_name = ssf.str();
    boost::filesystem::path dir_cls(dir_cls_name);
    boost::filesystem::create_directories(dir_cls);
   // cout << "cluster " << i << " : " << endl;
    for( size_t j = 0; j < clusters[i].members_ids.size(); j++ )
    {
     // cout << clusters[i].members_ids[j] << " , ";
      int mid = model_map[clusters[i].members_ids[j]];
      int id = model_local_map[clusters[i].members_ids[j]];
      
      CloudPtr cloud_deb(new Cloud);
      
      if( parts_ )
        cloud_deb = models[mid].surfaces_voxels[id];
      else
        cloud_deb = models[mid].segment_voxels[id];
      cloud_deb->width = 1;
      cloud_deb->height = cloud_deb->points.size();
      stringstream ss;
      //ss << "cluster" << i << "_" << models[mid].id << mid << id << ".pcd";
      //ss << dir_cls_name << "cluster" << i << "_" << models[mid].id << "_" << surface_map[mid] << "_" << voxel_map[mid] << "_part"<< ".pcd";
      ss << dir_cls_name << "cluster" << i << "_" << models[mid].id << "_part_" << surface_map[mid] << ".pcd";
      pcl::io::savePCDFileASCII(ss.str(),*cloud_deb);
      pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_cvp(new pcl::PointCloud<pcl::PrincipalCurvatures>);  
      Cube<double> cube_patch;
      Cube<double> cube_axis;
      if( !parts_ ){
        int vid = voxel_map[clusters[i].members_ids[j]];
        int sid = surface_map[clusters[i].members_ids[j]];
        cloud_cvp = models[mid].surface_voxel_cvts[sid][vid]; 
        cube_patch = models[mid].surface_patch_normal[sid][vid];
        cube_axis = models[mid].surface_patch_axis[sid][vid];
      }
      else
        cloud_cvp = models[mid].surface_cvts[id];
      stringstream ssv;
      cloud_cvp->width = 1;
      cloud_cvp->height = cloud_cvp->points.size();
      //ssv << dir_cls_name << "cluster" << i << "_" << models[mid].id << "_" << surface_map[mid] << "_" << voxel_map[id] << "_cvt"  << ".pcd";
      ssv << dir_cls_name << "cluster" << i << "_" << models[mid].id << "_cvt_" << surface_map[mid] << ".pcd";
      //ssv << dir_cls_name << "cluster" << i << "_" << models[mid].id << "_" << mid << "_" << id << "_cvt"  << ".pcd";
      pcl::io::savePCDFileASCII (ssv.str(),*cloud_cvp);

      CloudPtr cloud_patch(new Cloud);
      for( int ip = 0; ip < cube_patch.n_elem; ip++ )
      {
        PointType p;
        p.x = cube_patch(ip); p.y = 0; p.z = 0;
        cloud_patch->points.push_back(p);
      }

      cloud_patch->width = 1;
      cloud_patch->height = cloud_patch->points.size();
      stringstream ss_p;
      ss_p << dir_cls_name << "cluster" << i << "_" << models[mid].id << "_" << surface_map[mid] << "_normal_patch" << voxel_map[id] << ".pcd";
      pcl::io::savePCDFileASCII (ss_p.str(),*cloud_patch);

      CloudPtr cloud_axis(new Cloud);
      for( int ip = 0; ip < cube_axis.n_elem; ip++ )
      {
        PointType p;
        p.x = cube_axis(ip); p.y = 0; p.z = 0;
        cloud_axis->points.push_back(p);
      }
      if( cloud_axis->points.size() == 0 )
        continue;
      cloud_axis->width = 1;
      cloud_axis->height = cloud_axis->points.size();
      stringstream ss_ax;
      ss_ax << dir_cls_name << "cluster" << i << "_" << models[mid].id << "_" << surface_map[mid] << "_axis_patch" << voxel_map[id] << ".pcd";
      pcl::io::savePCDFileASCII (ss_ax.str(),*cloud_axis);

    }
    //cout << endl;
  }
  
  return clusters;
}


vector<model> vision::make_evaluation_models(string eval_path)
{
  vector<model> models;
  vector<string> files = get_all_files(eval_path,".pcd");  
  for( size_t i = 0; i < files.size(); i++ )
  {
    cout << "make model: " << i << " : " << files.size() << endl;
    CloudPtr cloud = load_cloud(files[i]);
		if(cloud->points.size() == 0 )
			continue;
		cout << "to decompose model" << endl;
    model m = decompose_object(cloud);
    extract_feature(m);
    extract_feature_patch(m);
    extract_feature_axis(m);
    m.id = file_name_;
   // make_histogram(m,false);
    models.push_back(m);    
  }
	cout << "eval models made" << endl;
  return models;
}

model vision::make_evaluation_model(string eval_path)
{
	cout << "make model:" << eval_path << endl;
  CloudPtr cloud = load_cloud(eval_path);
	model m;
  if(cloud->points.size() == 0 )
    return m;
  cout << "to decompose model" << endl;

  m = decompose_object(cloud);
  extract_feature(m);
  extract_feature_patch(m);
  extract_feature_axis(m);
  m.id = file_name_;
   // make_histogram(m,false);
  return m;
}

void vision::estimate_codebook_frequency(vector<Hcluster> clusters)
{
  map<string,vector<model> >::iterator it;
  for( it = model_prototypes_.begin(); it != model_prototypes_.end(); it++ ) 
  {
    vector<double> vec(clusters.size(),0);
    prototype_frequency_[(*it).first] = vec;
    prototype_occ_[(*it).first] = vec;
  }

  for( it = model_prototypes_.begin(); it != model_prototypes_.end(); it++ )
  {
    vector<model> models = (*it).second;
    int num_data = 0;
    for( size_t i = 0; i < models.size(); i++ )
    {
      map<int,vector<Cube<double> > >::iterator itm; 
      //int num_data = 0;
      for( itm = models[i].surface_voxel_hist.begin(); itm != models[i].surface_voxel_hist.end(); itm++ )
      {
        vector<Cube<double> > hists = (*itm).second;
        num_data += hists.size();
        for( size_t ih = 0; ih < hists.size(); ih++ )
        {
          vector<int> ids = measure_similarity_codebook(clusters,hists[ih]);
          for( size_t j = 0; j < ids.size(); j++ ){
            prototype_frequency_[(*it).first][ids[j]] ++;
            prototype_occ_[(*it).first][ids[j]] ++;
          }
        }
      }
      //for( size_t j = 0; j < clusters.size(); j++ )
      //  prototype_frequency_[(*it).first][j] /= num_data;
    }
    for( size_t j = 0; j < clusters.size(); j++ )
      prototype_frequency_[(*it).first][j] /= num_data;
  }
  map<string,vector<double> >::iterator its;
  for( its = prototype_frequency_.begin(); its != prototype_frequency_.end(); its++ )
  {
    cout << (*its).first << endl;
    vector<double> vals = (*its).second;
    for( size_t i = 0; i < vals.size(); i++ )
      cout << vals[i] << " , ";
    cout << endl;
  }
 // for( size_t i = 0; i < clusters.size(); i++ )
//   cout << "distance: " << clusters[i].dist_threshold << endl;
}

vector<int> vision::measure_similarity_codebook(vector<Hcluster> clusters,Cube<double> hist)
{
  double epsilon = 1e-10;
  vector<int> ids;
	dists_cb_.clear();
	dists_cb_all_.clear();
	ids_all_.clear();
  for( size_t i = 0; i < clusters.size(); i++ )
  {
    double dist;
  	if( parts_cls_)
  		dist = chi_hist_distance(clusters[i].center,hist);
  	else
  		dist = shape_context_distance(clusters[i].center,hist);
		dists_cb_all_.push_back(dist);
		ids_all_.push_back(i);
    if( dist <= clusters[i].dist_threshold ){			
      ids.push_back(i);
      dists_cb_.push_back(dist);
    }
  }
	
  return ids;
}

vector<set<int> >  vision::evaluate_model_patch(model m,vector<Hcluster> clusters)
{
	vector<double> regs_prob_cur;
  set<int> seen_ids;
  vector<set<int> > regions;
 // double epsilon = 1e-10;
//  double epsilon = 0.5 - 1e-01;

//  double epsilon = min_potential_;

	double epsilon = prob_thresh_;
//	epsilon = 0.5 - 1e-5;

	potential_ = 0;
  while( seen_ids.size() < m.surfaces.size() )
  {
    int id;
    do{
      id = rand() % m.surfaces.size();
    }while( seen_ids.find(id) != seen_ids.end() );
    seen_ids.insert(id);
    set<int> reg;
    reg.insert(id);
    set<int>::iterator its;
    set<int> reg_nns;
    int best_id = -1;
    double max_dist = 0;

    for( its = reg.begin(); its != reg.end(); its++ )
    {
      set<int> nns = m.surfaces_nns[*its];
      set<int>::iterator its_2;
      for( its_2 = nns.begin(); its_2 != nns.end(); its_2++ )
      {
        vector<int> cls_cur = measure_similarity_codebook(clusters,m.surface_patch_axis[(*its_2)][0]);
				vector<double> cur_probs = dists_cb_;
				cout << "cls cur size: " << cls_cur.size() << endl;
        patch_cb_[(*its_2)] = cls_cur;
        double dist_all = 0;
        set<int>::iterator its_ref;
        int count_p = 0;
        for( its_ref = reg.begin(); its_ref != reg.end(); its_ref++ )
        {
          double dist = 0;
					double dist_n = 0;
          if( m.surfaces_nns[*its_ref].find(*its_2) == m.surfaces_nns[*its_ref].end() )
            continue;
          count_p ++;
          vector<int> cls_ref = measure_similarity_codebook(clusters,m.surface_patch_axis[(*its)][0]);
					vector<double> ref_probs = dists_cb_;
					cout << "cls ref size: " << cls_ref.size() << endl;
          patch_cb_[(*its)] = cls_ref;
          for( size_t i = 0; i < cls_ref.size(); i++ )
          {
            for( size_t j = 0; j < cls_cur.size(); j++ )
            {
							double cur_dist;
							cur_dist = occ_mat_(cls_ref[i],cls_cur[j]);
              dist += cur_dist;
							
							double cur_dist_n;
							cur_dist_n = patch_occ_n_(cls_ref[i],cls_cur[j]);
							dist_n += cur_dist_n;
            }
          }
					if( cls_ref.size() * cls_cur.size() > 0 ){
						dist /= (cls_ref.size() * cls_cur.size());
						dist_n /= (cls_ref.size() * cls_cur.size());
					}
					dist /= (dist+dist_n);
          dist_all += dist;
        }
				if( dist_all > potential_ )
					potential_ = dist_all;
        if( dist_all > max_dist )
        {
          max_dist = dist_all;
          best_id = (*its_2);
        }
      }
    }
    cout << "cur prob: " << max_dist << endl;
    if( ( max_dist > epsilon ) && ( seen_ids.find(best_id) == seen_ids.end() ) )
    {
      cout << "prob to merge: " << max_dist << endl;
      seen_ids.insert(best_id);
      reg_nns.insert(best_id);
			regs_prob_cur.push_back(max_dist);
    }
		else
			regs_prob_cur.push_back(epsilon);

    for( its = reg_nns.begin(); its != reg_nns.end(); its++ )
      reg.insert(*its);
    regions.push_back(reg);
  }
  
	regs_prob_ = regs_prob_cur;
  cout << "regions size: " << regions.size() << endl;
  ColorCloudPtr cloud_deb(new ColorCloud);
  for( size_t i = 0; i < regions.size(); i++ ) 
  {
    double r = rand() % 255; double g = rand() % 255; double b = rand() % 255;
    set<int>::iterator its;
    cout << regions[i].size() << endl;
    for( its = regions[i].begin(); its != regions[i].end(); its++ )
    {
      CloudPtr cloud_cur = m.surfaces_voxels[*its];
      for( size_t ip = 0; ip < cloud_cur->points.size(); ip++ )
      {
        ColorPointType p;
        p.x = cloud_cur->points[ip].x; p.y = cloud_cur->points[ip].y; p.z = cloud_cur->points[ip].z;
        p.r = r; p.g = g; p.b = b;
        cloud_deb->points.push_back(p);
      }
    }
  }

  if( debug_eval_ ){
    pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("cloud viewer regions merged init");
    viewer->addPointCloud(cloud_deb);

    while(!viewer->wasStopped())
      viewer->spinOnce (100);
    delete viewer;
  }
  return regions;
}

void vision::visualize_surfaces(model m)
{
  ColorCloudPtr cloud_deb(new ColorCloud);
  map<int,CloudPtr>::iterator it;
  for( it = m.surfaces_voxels.begin(); it != m.surfaces_voxels.end(); it++ )
  {
    CloudPtr cloud_cur = (*it).second;
    double r = rand() % 255; double g = rand() % 255; double b = rand() % 255;
    for( size_t ip = 0; ip < cloud_cur->points.size(); ip++ )
    {
      ColorPointType p;
      p.x = cloud_cur->points[ip].x; p.y = cloud_cur->points[ip].y; p.z = cloud_cur->points[ip].z;
      p.r = r; p.g = g; p.b = b;
      cloud_deb->points.push_back(p);
    }
  }

  pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("before region growing");
  viewer->addPointCloud(cloud_deb);

  while(!viewer->wasStopped())
    viewer->spinOnce (100);
 // delete viewer;  
}

vector<set<int> > vision::evaluate_model_patch_new(model m,vector<Hcluster> clusters,vector<set<int> > regs)
{
	vector<double> reg_prob_cur;
  vector<set<int> > regions;
  set<int> seen_ids;
  vector<set<int> > parts_h_vec;
  double num_seg = 0;
  map<int,vector<Cube<double> > >::iterator it_sp;
  for( it_sp = m.surface_patch_normal.begin(); it_sp != m.surface_patch_normal.end(); it_sp++ )
    num_seg+= m.surface_patch_normal[(*it_sp).first].size();
  double epsilon = 0.5 - 1e-1;
  epsilon = 1e-5;
  //epsilon = min_potential_;
	double thresh = prob_thresh_;
	//thresh = 0.5 - 1e-5;

	potential_ = FLT_MAX;

  for( size_t i = 0; i < regs.size(); i++ )
  {
    if( seen_ids.find(i) != seen_ids.end() )
      continue;
    set<int> reg;
    reg.insert(i);
    seen_ids.insert(i);
    double dist_all = 0;
    int best_id = -1;
		for( size_t j = 0; j < regs.size(); j++ )
//		for( size_t j = i+1; j < regs.size(); j++ )
    {
			if( j == i )
				continue;
      set<int>::iterator its,its_r;
      double cur_dist_re = 0;
			double n_cur_dist_re = 0;

      int count_p = 0;
      double bias_term = 0;
      for( its_r = regs[i].begin(); its_r != regs[i].end(); its_r++ )
      {
				count_p = 0;
        vector<int> cls_ref = measure_similarity_codebook(clusters,m.surface_patch_axis[(*its_r)][0]);
				vector<double> ref_probs = dists_cb_;
				cout << "ref patch: " << cls_ref.size() << endl;
        set<int>::iterator its_rn;
        double bias_term_local = 0;
				  bias_term_local = 1. / num_seg;
        bias_term = num_seg;
        double cur_dist = 0;
				double n_cur_dist = 0;
        //** \prod_{z_{k}} p(z_{k} | z_{j})**//
        for( its = regs[j].begin(); its != regs[j].end(); its++ )
        {
          if( m.surfaces_nns[*its_r].find(*its) == m.surfaces_nns[*its_r].end() )
            continue;
					count_p++;
          vector<int> cls_cur = measure_similarity_codebook(clusters,m.surface_patch_axis[(*its)][0]);
					vector<double> cur_probs = dists_cb_;
					cout << "test patch: " << cls_cur.size() << endl;
          double dist = 0;
					double n_dist = 0;
          for( size_t ir = 0; ir < cls_ref.size(); ir++ )
          {
            for( size_t ic = 0; ic < cls_cur.size(); ic++ ){
							dist += occ_mat_(cls_ref[ir],cls_cur[ic]);

							n_dist += patch_occ_n_(cls_ref[ir],cls_cur[ic]);
						}
          }
					if( cls_ref.size() * cls_cur.size() > 0 ){
						dist /= (double)(cls_ref.size() * cls_cur.size());
						n_dist /= (double)(cls_ref.size() * cls_cur.size());
					}
				  if( dist > epsilon )
					{
						dist /= bias_term_local;
						n_dist /= bias_term_local;
					}
          if(count_p == 1 ){
            cur_dist = dist;	
						n_cur_dist = n_dist;
					}
          else 
					{
            cur_dist *= dist;   
						n_cur_dist *= n_dist;
           /* cur_dist += dist;
            n_cur_dist += n_dist;		*/				
					}
        }
        // ** //
        if( count_p < 1 ) // ** reference patch has no neighbor in other region** //
          continue;
        double cur_dist_n = 0;
				double n_cur_dist_n = 0;
        count_p = 0;
        for( its_rn = m.surfaces_nns[*its_r].begin(); its_rn != m.surfaces_nns[*its_r].end(); its_rn++ )
        {
          if( regs[i].find(*its_rn) == regs[i].end() )
            continue;
					count_p ++;
          double dist_n = 0;
					double n_dist_n = 0;
          vector<int> cls_refn = measure_similarity_codebook(clusters,m.surface_patch_axis[(*its_rn)][0]);
          for( size_t ir = 0; ir < cls_ref.size(); ir++ )
          {
            for( size_t ic = 0; ic < cls_refn.size(); ic++ ){
              dist_n += occ_mat_(cls_ref[ir],cls_refn[ic]);
							n_dist_n += patch_occ_n_(cls_ref[ir],cls_refn[ic]);
						}
          }
					if( cls_ref.size() * cls_refn.size() > 0 ){
						dist_n /= (double)(cls_ref.size() * cls_refn.size());
						n_dist_n /= (double)(cls_ref.size() * cls_refn.size());
					}
					if( n_dist_n > epsilon )
						n_dist_n /= bias_term_local;
					if( dist_n > epsilon )
						dist_n /= bias_term_local;
          if( count_p == 1 ){
            cur_dist_n = dist_n;
						n_cur_dist_n = n_dist_n;
					}
          else{
            cur_dist_n *= dist_n;  
						n_cur_dist_n *= n_dist_n;  

         /*   cur_dist_n += dist_n;
            n_cur_dist_n += n_dist_n;*/
					}     
        }
        if( count_p > 0 ){
          cur_dist *= cur_dist_n;	
					n_cur_dist *= n_cur_dist_n;
				}
        cur_dist /= num_seg;
				n_cur_dist /= num_seg;

        cur_dist_re += cur_dist;
				n_cur_dist_re += n_cur_dist;
      }
			cur_dist_re /= (cur_dist_re+n_cur_dist_re);
      if( cur_dist_re > dist_all )
      {
        dist_all = cur_dist_re;
        best_id = j;
      }
    }
	
    cout << "prob: " << dist_all << endl;
		if( ( dist_all > thresh ) && ( ( seen_ids.find(best_id) == seen_ids.end() ) && ( reg.find(best_id) == reg.end() ) ) && (best_id >= 0) )
    {
			cout << "prob merge: " << dist_all << endl;
			reg.insert(best_id);
			seen_ids.insert(best_id);
			reg_prob_cur.push_back(dist_all);

			if( dist_all < potential_ )
				potential_ = dist_all;
    }
		else
			reg_prob_cur.push_back(regs_prob_[i]);

    set<int>::iterator its;
    set<int> region;
    for( its = reg.begin(); its != reg.end(); its++ )
    {
      set<int>::iterator its_2;
      for( its_2 = regs[*its].begin(); its_2 != regs[*its].end(); its_2++ ){
        region.insert(*its_2);
      }
    }
    parts_h_vec.push_back(reg);
    regions.push_back(region); 
  }
  parts_h_.push_back(parts_h_vec);

	regs_prob_ = reg_prob_cur;
  cout << "regions size: " << regions.size() << endl;
  ColorCloudPtr cloud_deb(new ColorCloud);
  for( size_t i = 0; i < regions.size(); i++ )
  {
    double r = rand() % 255; double g = rand() % 255; double b = rand() % 255;
    set<int>::iterator its;
    cout << regions[i].size() << endl;
    for( its = regions[i].begin(); its != regions[i].end(); its++ )
    {
      CloudPtr cloud_cur = m.surfaces_voxels[*its];
      for( size_t ip = 0; ip < cloud_cur->points.size(); ip++ )
      {
        ColorPointType p;
        p.x = cloud_cur->points[ip].x; p.y = cloud_cur->points[ip].y; p.z = cloud_cur->points[ip].z;
        p.r = r; p.g = g; p.b = b;
        cloud_deb->points.push_back(p);
      }
    }
  }

  if( debug_eval_ ){
    pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("cloud viewer regions merged");
    viewer->addPointCloud(cloud_deb);

    while(!viewer->wasStopped())
      viewer->spinOnce (100);
    delete viewer;
  }

  return regions;
}

vector<set<int> > vision::evaluate_model_patch(model m,vector<Hcluster> clusters,vector<set<int> > regs)
{
	vector<double> regs_prob_cur;

  vector<set<int> > regions;
  set<int> seen_ids;
//  double epsilon = 0.5 - 1e-1;
	double epsilon = min_potential_;
  for( size_t i = 0; i < regs.size(); i++ )
  {
    if( seen_ids.find(i) != seen_ids.end() )
      continue;
    set<int> reg;
    reg.insert(i);
    seen_ids.insert(i);
    double dist_all = 0;
    int best_id = -1;
    for( size_t j = i+1; j < regs.size(); j++ )
    {
      set<int>::iterator its;
      double cur_dist = 0;
      int count_p = 0;
      //double bias_term = 0;
      //double bias_nn = 0;
      double bias_term = 0;
      for( its = regs[j].begin(); its != regs[j].end(); its++ )      
      {
        //vector<int> cls_cur = measure_similarity_codebook(clusters,m.surface_patch_normal[(*its)][0]);
         vector<int> cls_cur = measure_similarity_codebook(clusters,m.surface_patch_axis[(*its)][0]);
        set<int>::iterator its_r;
        for( its_r = regs[i].begin(); its_r != regs[i].end(); its_r++ )
        {
          if( m.surfaces_nns[*its_r].find(*its) == m.surfaces_nns[*its_r].end() )
            continue;
          vector<int> cls_ref = measure_similarity_codebook(clusters,m.surface_patch_normal[(*its_r)][0]);
          double bias_term_local = 0;
          for( size_t ic_r = 0; ic_r < cls_ref.size(); ic_r++ )
            bias_term_local += accu(occ_mat_.row(cls_ref[ic_r]));
          set<int>::iterator its_rn;
          bias_term = 0;
          for( its_rn = m.surfaces_nns[*its_r].begin(); its_rn != m.surfaces_nns[*its_r].end(); its_rn++ )
          {
            if( ( regs[i].find(*its_rn) == regs[i].end() ) && ( regs[j].find(*its_rn) == regs[j].end() ) )
              continue;
            //vector<int> cls_refn = measure_similarity_codebook(clusters,m.surface_patch_normal[(*its_rn)][0]);
            vector<int> cls_refn = measure_similarity_codebook(clusters,m.surface_patch_axis[(*its_rn)][0]);
            double cur_distn = 0;
            for( size_t ir = 0; ir < cls_ref.size(); ir++ )
            {
              for( size_t ic = 0; ic < cls_refn.size(); ic++ )
              {
                cur_distn += occ_mat_(cls_ref[ir],cls_refn[ic]);
              }
            }
         //   cur_distn /= bias_term_local;
            bias_term += cur_distn;
          }
          double dist = 0;
          for( size_t ir = 0; ir < cls_ref.size(); ir++ )
          {
            for( size_t ic = 0; ic < cls_cur.size(); ic++ )
              dist += occ_mat_(cls_ref[ir],cls_cur[ic]);
          }
          //dist /= bias_term_local;
          dist /= bias_term;
        //  bias_term += dist;
          
          double cur_dist_n = 1;
          for( its_rn = m.surfaces_nns[*its_r].begin(); its_rn != m.surfaces_nns[*its_r].end(); its_rn++ )
          {
            if( regs[i].find(*its_rn) == regs[i].end() )
              continue;
            double dist_n = 0;
            //vector<int> cls_refn = measure_similarity_codebook(clusters,m.surface_patch_normal[(*its_rn)][0]);
            vector<int> cls_refn = measure_similarity_codebook(clusters,m.surface_patch_axis[(*its_rn)][0]); 
            for( size_t ir = 0; ir < cls_ref.size(); ir++ )
            {
              for( size_t ic = 0; ic < cls_refn.size(); ic++ )
                dist_n += occ_mat_(cls_ref[ir],cls_refn[ic]);
            }
            dist_n /= bias_term;
            //dist_n /= bias_term_local;
            cur_dist_n *= dist_n;         
           // bias_nn += dist_n;
          }
          dist *= cur_dist_n;
          //dist /= bias_term;

          count_p++;
          if( count_p == 1 )
            cur_dist = dist;
          else
            //cur_dist *= dist;
            cur_dist += dist;
        }
      }
      //bias_term *= bias_nn;
      //cur_dist /= bias_term;
      if( cur_dist > dist_all )
      {
        dist_all = cur_dist;
        best_id = j;
      }
    }
    cout << "prob: " << dist_all << endl;
    if( ( dist_all >= epsilon ) && ( ( seen_ids.find(best_id) == seen_ids.end() ) || ( reg.find(best_id) == reg.end() ) ) )
    {
      cout << "prob merge: " << dist_all << endl;
      reg.insert(best_id);
      seen_ids.insert(best_id);
			regs_prob_cur.push_back(dist_all);
    }
		else
			regs_prob_cur.push_back(epsilon);

    set<int>::iterator its;
    set<int> region;
    for( its = reg.begin(); its != reg.end(); its++ )
    {
      set<int>::iterator its_2;
      for( its_2 = regs[*its].begin(); its_2 != regs[*its].end(); its_2++ )
        region.insert(*its_2);
    }
    regions.push_back(region); 
  }

	regs_prob_ = regs_prob_cur;
  cout << "regions size: " << regions.size() << endl;
  ColorCloudPtr cloud_deb(new ColorCloud);
  for( size_t i = 0; i < regions.size(); i++ )
  {
    double r = rand() % 255; double g = rand() % 255; double b = rand() % 255;
    set<int>::iterator its;
    cout << regions[i].size() << endl;
    for( its = regions[i].begin(); its != regions[i].end(); its++ )
    {
      CloudPtr cloud_cur = m.surfaces_voxels[*its];
      for( size_t ip = 0; ip < cloud_cur->points.size(); ip++ )
      {
        ColorPointType p;
        p.x = cloud_cur->points[ip].x; p.y = cloud_cur->points[ip].y; p.z = cloud_cur->points[ip].z;
        p.r = r; p.g = g; p.b = b;
        cloud_deb->points.push_back(p);
      }
    }
  }

  //if( debug_ ){
    pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("cloud viewer regions merged");
    viewer->addPointCloud(cloud_deb);

    while(!viewer->wasStopped())
      viewer->spinOnce (100);
    //delete viewer;
  //}

  return regions;
}


void vision::make_parts_visualization(vector<vector<set<int> > > parts_h,vector<vector<set<int> > > parts_h_pts,model m)
{
  cout << "to save visulization" << endl;
  stringstream ss_part;
	ss_part << default_path_ << "vis-parts/" << m.id << "/";
  string dir_patch_name = ss_part.str();
  boost::filesystem::path dir_patch(dir_patch_name);
  boost::filesystem::create_directories(dir_patch);

  search::KdTree<PointType>::Ptr tree (new search::KdTree<PointType>);
  tree->setInputCloud(m.cloud);
  vector< int > k_indices;
  vector< float > k_sqr_distances;
	int k = 2;

	map<int,set<int> >::iterator its;
	for( its = m.surfaces_nns.begin(); its != m.surfaces_nns.end(); its++ )
	{
		set<int> nns = (*its).second;
		int id = (*its).first;
		ColorCloudPtr cloud_d(new ColorCloud);
		set<int> ids_seen;
		for( size_t i = 0; i < m.surfaces_voxels[id]->points.size(); i++ )
		{
			PointType p = m.surfaces_voxels[id]->points[i];
			tree->nearestKSearch(p,k,k_indices,k_sqr_distances);
			ids_seen.insert(k_indices[0]);
		}
	
		set<int>::iterator it_n;
		for( it_n = nns.begin(); it_n != nns.end(); it_n++ )
		{
			CloudPtr cloud_nn = m.surfaces_voxels[(*it_n)];
			for( size_t i = 0; i < cloud_nn->points.size(); i++ )
			{
				PointType p = cloud_nn->points[i];
				tree->nearestKSearch(p,k,k_indices,k_sqr_distances);
				ids_seen.insert(k_indices[0]);
			}
		}
		
		for( size_t i = 0; i < m.cloud->points.size(); i++ )
		{
			ColorPointType p;
			p.x = m.cloud->points[i].x; p.y = m.cloud->points[i].y; p.z = m.cloud->points[i].z;
			p.r = 255; p.g = 255; p.b = 255;
			if( ids_seen.find(i) != ids_seen.end() )
			{
				p.g = 0; p.b = 0;
			}
			cloud_d->points.push_back(p);		
		}

    stringstream ss_p;
    string sname;
    ss_p << dir_patch_name << "nn" << id;
    sname = ss_p.str();
    if( cloud_d->points.size() == 0 )
      continue;
    sname = sname + ".pcd";
    cloud_d->width = 1;
    cloud_d->height = cloud_d->points.size();
    pcl::io::savePCDFileASCII(sname,*cloud_d);
	}
	
	map< uint32_t,Supervoxel<PointType>::Ptr >::iterator it_sc;
  for( it_sc = m.supervoxel_clusters.begin(); it_sc != m.supervoxel_clusters.end(); it_sc++ )
  {
    int id = (*it_sc).first;
    if( m.supervoxel_clusters[id]->voxels_->points.size() > 10 )
    {
			CloudPtr cloud = m.supervoxel_clusters[id]->voxels_;
			stringstream ss_p;
			string sname;
			ss_p << dir_patch_name << "s" << id;
			sname = ss_p.str();
			if( cloud->points.size() == 0 )
				continue;
			sname = sname + ".pcd";
			cloud->width = 1;
			cloud->height = cloud->points.size();
			pcl::io::savePCDFileASCII(sname,*cloud);			
    }
	}

	map<int,CloudPtr>::iterator it;
	for( it = m.surfaces_voxels.begin(); it != m.surfaces_voxels.end(); it++ )
	{
		CloudPtr cloud = (*it).second;
    stringstream ss_p;
    string sname;
    ss_p << dir_patch_name << "l" << "_r" << (*it).first;
    sname = ss_p.str();		
    if( cloud->points.size() == 0 )
      continue;
    sname = sname + ".pcd";
    cloud->width = 1;
    cloud->height = cloud->points.size();
    pcl::io::savePCDFileASCII(sname,*cloud);
	}

  for( size_t ih = 0; ih < parts_h_pts.size(); ih++ ) 
  {
    for( size_t i = 0; i < parts_h[ih].size(); i++ )
    {
      CloudPtr cloud(new Cloud);
      set<int>::iterator its;
      for( its = parts_h_pts[ih][i].begin(); its != parts_h_pts[ih][i].end(); its++ )
      {
        if( m.surfaces_voxels.find(*its) == m.surfaces_voxels.end() )
          continue;
        CloudPtr cloud_cur = m.surfaces_voxels[*its];
        for( size_t ip = 0; ip < cloud_cur->points.size(); ip++ )
          cloud->points.push_back(cloud_cur->points[ip]);
      }
      stringstream ss_p;
      string sname;
      ss_p << dir_patch_name << "l" << ih << "_r" << i;
      sname = ss_p.str();
      for( its = parts_h[ih][i].begin(); its != parts_h[ih][i].end(); its++ ){
        stringstream ss;
        ss << "_" << (*its);
        sname = sname + ss.str();
      }
      if( cloud->points.size() == 0 )
        continue;
      sname = sname + ".pcd";
      cloud->width = 1;
      cloud->height = cloud->points.size();
      pcl::io::savePCDFileASCII(sname,*cloud);       
    }
  }

}

vector<CloudPtr> vision::extract_part_pts(model m,vector<set<int> > regs)
{
	string dir_patch_name = default_path_ + "eval-parts/";
  boost::filesystem::path dir_patch(dir_patch_name);
  boost::filesystem::create_directories(dir_patch);

  ColorCloudPtr cloud_deb(new ColorCloud);
  vector<CloudPtr> clouds;
  for( size_t i = 0; i < regs.size(); i++ )
  {
		string dir_patch_cur = dir_patch_name + m.id + "/";
		boost::filesystem::path dir_cur(dir_patch_cur);
		boost::filesystem::create_directories(dir_cur);

    CloudPtr cloud(new Cloud);
    set<int>::iterator its;
    double r = rand() % 255; double g = rand() % 255; double b = rand() % 255;
    for( its = regs[i].begin(); its != regs[i].end(); its++ )
    {
      CloudPtr cloud_cur = m.surfaces_voxels[*its];
      for( size_t ip = 0; ip < cloud_cur->points.size(); ip++ ){
        cloud->points.push_back(cloud_cur->points[ip]);
        ColorPointType p;
        p.r = r; p.g = g; p.b = b;
        p.x = cloud_cur->points[ip].x; p.y = cloud_cur->points[ip].y; p.z = cloud_cur->points[ip].z;
        cloud_deb->points.push_back(p);
      }
    }
    stringstream ssv;
    cloud->width = 1;
    cloud->height = cloud->points.size();
		ssv << dir_patch_cur << m.id << "_patch" << i << ".pcd";
   // ssv << dir_patch_name << m.id << "_patch" << i << ".pcd";
    pcl::io::savePCDFileASCII (ssv.str(),*cloud);

		Cube<double> part_hist = m.parts_hist[i];
		Mat<double> part_fet(1,part_hist.n_elem); 		
		for(int ih = 0; ih < part_hist.n_elem; ih++)
			part_fet(0,ih) = part_hist(ih);
		stringstream sf;
		sf << dir_patch_cur << m.id << "_patch" << i  << "_hist.mat";
		string path_mat = sf.str();
		part_fet.save(path_mat,arma_ascii);			

    clouds.push_back(cloud);
  }
  if( debug_eval_ ){
    pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("cloud viewer best regions merged");
    viewer->addPointCloud(cloud_deb);

    while(!viewer->wasStopped())
      viewer->spinOnce (100);
    delete viewer;
  }
  return clouds; 
}

double vision::estimate_overlap(vector<CloudPtr> parts,model m)
{
	p_ids_.clear();
	p_overlap_.clear();
  double prob = 0;
  string pattern = m.id;
  pattern = pattern + "_patch";
 // string eval_dir_name = "/media/TOSHIBA\ EXT/eval-ground-truth/";
	string eval_dir_name = default_path_ + "eval-ground-truth/";
  cout << "pattern: " << pattern <<endl;
  cout << "dir: " << eval_dir_name << endl;
  vector<string> eval_files = get_all_files(eval_dir_name,pattern);
  vector<CloudPtr> eval_parts;
  CloudPtr cloud_all(new Cloud);
  for( size_t i = 0; i < parts.size(); i++ )
  {
    for( size_t ip = 0; ip < parts[i]->points.size(); ip++ )
      cloud_all->points.push_back(parts[i]->points[ip]);
  }
  
  double epsilon = 1e-2;
 // double epsilon = 0.05;
	cout << "parts size: " << parts.size() << endl;
	cout << "to make init kdtree" << cloud_all->points.size() << endl;
  search::KdTree<PointType>::Ptr tree_all (new search::KdTree<PointType>);
  tree_all->setInputCloud(cloud_all);
	cout << "afetr init kdtree" << endl;
  vector< int > k_ids; vector< float > k_dist;
  int k = 2;

  for( size_t i = 0; i < eval_files.size(); i++ )
  {
		string str = eval_files[i];
		cout << str << endl;
		string cue = "_patch";
		// only for washington mugs //
//		str = str.substr(str.find(cue)+cue.length()+1);
		// for others //
		 str = str.substr(str.find(cue)+cue.length());
		cout << str << endl;
		str = str.substr(0,str.find_last_of("."));
		cout << str << endl;
		int p_id;
		stringstream ss(str);
		ss >> p_id;
		cout << "pid: " << p_id << endl;
		p_ids_.push_back(p_id);

    set<int> pts_set;
    CloudPtr cloud = load_cloud(eval_files[i]);
    CloudPtr cloud_tmp(new Cloud);
    for( size_t ip = 0; ip < cloud->points.size(); ip++ )
    {
      tree_all->nearestKSearch(cloud->points[ip],k,k_ids,k_dist);
      pts_set.insert(k_ids[0]);
      //if( k_dist[0] <= epsilon )
        //cloud_tmp->points.push_back(cloud_all->points[k_ids[0]]);
    }
    set<int>::iterator its;
    for( its = pts_set.begin(); its != pts_set.end(); its++ )
      cloud_tmp->points.push_back(cloud_all->points[*its]);
    //eval_parts.push_back(cloud);
    eval_parts.push_back(cloud_tmp);
  }

	if( eval_parts.size() == 0 )
		return -1;
	CloudPtr cloud_org(new Cloud);
	for( size_t i = 0; i < parts.size(); i++ )
	{
		for( size_t j = 0; j < parts[i]->points.size(); j++ )
		{
			cloud_org->points.push_back(parts[i]->points[j]);
		}
	}
	epsilon = 1e-2;

	cout << "to make original kd tree" << endl;
	cout  << cloud_org->points.size() << endl;
	search::KdTree<PointType>::Ptr tree_org (new search::KdTree<PointType>);
	tree_org->setInputCloud(cloud_org);
	cout << "tree built" << endl;
	CloudPtr cloud_ref(new Cloud);
	for( size_t i = 0; i < eval_parts.size(); i++ )	
	{
		vector< int > k_indices; vector< float > k_sqr_distances;
		int k = 2;
		CloudPtr cur_cloud(new Cloud);
		for( size_t j = 0; j < eval_parts[i]->points.size(); j++ )
		{
			tree_org->nearestKSearch(eval_parts[i]->points[j],k,k_indices,k_sqr_distances);
			if( k_sqr_distances[0] <= epsilon ){
				cur_cloud->points.push_back(eval_parts[i]->points[j]);
				cloud_ref->points.push_back(eval_parts[i]->points[j]);
			}
		}
		*eval_parts[i] = *cur_cloud;
	}
	cout << "after original kdtree: " << cloud_ref->points.size() << endl;

	epsilon = 1e-10;
	tree_org->setInputCloud(cloud_ref);
	for( size_t i = 0; i < parts.size(); i++ )
	{
		vector< int > k_indices; vector< float > k_sqr_distances;
		int k = 2;
		CloudPtr cur_cloud(new Cloud);
		for( size_t j = 0; j < parts[i]->points.size(); j++ )
		{
			tree_org->nearestKSearch(parts[i]->points[j],k,k_indices,k_sqr_distances);
      if( k_sqr_distances[0] <= epsilon )
        cur_cloud->points.push_back(parts[i]->points[j]);			
		}
		cout << "size: "<< parts[i]->points.size() << " : " << cur_cloud->points.size() << endl;
		*parts[i] = *cur_cloud;
	}

  vector<double> probs;
	int min_pts = 10;
  double radius = 0.05;
  for( size_t i = 0; i < eval_parts.size(); i++ ) 
  {
    double max_prob = 0; 
    vector<double> probs_local;
    bool found = false;
    double min_prob = 0.1;
		double p_prob = 0;
		int p_count = 0;
    for( size_t j = 0; j < parts.size(); j++ )
    {
			if( parts[j]->points.size() < min_pts )
				continue;
      int count_p = 0;
      search::KdTree<PointType>::Ptr tree (new search::KdTree<PointType>);
      tree->setInputCloud(parts[j]);
      vector< int > k_indices; vector< float > k_sqr_distances;
      int k = 2;
      set<int> pts_set;
      for( size_t ip = 0; ip < eval_parts[i]->points.size(); ip++ ){
        tree->nearestKSearch(eval_parts[i]->points[ip],k,k_indices,k_sqr_distances);  
        if( k_sqr_distances[0] <= epsilon ){
          count_p ++;
          pts_set.insert(k_indices[0]);
        }
        /*tree->radiusSearch(eval_parts[i]->points[ip],radius,k_indices,k_sqr_distances);
        for( size_t ik = 0; ik < k_indices.size(); ik++ )
          pts_set.insert(k_indices[ik]);*/
      }
      count_p = pts_set.size();
      cout << "count_p: " << count_p << " : " << eval_parts[i]->points.size() << " , " << parts[j]->points.size() << endl;
			//double thresh = (double) eval_parts[i]->points.size() * 0.1;
      if( count_p > 0 ){
        found = true;
        double prob_cur = double(count_p) / double (eval_parts[i]->points.size() + parts[j]->points.size() - count_p);
        if( prob_cur > 1 )
          prob_cur = 1;
        if( prob_cur > max_prob )
          max_prob = prob_cur;
        probs_local.push_back(prob_cur);
        /*if( prob_cur > min_prob ){
					//p_prob += prob_cur;
					//p_count++;
          probs.push_back(prob_cur);
				}*/
      }
    }
		cout << "max prob is: " << max_prob << endl;
		probs.push_back(max_prob);
  }

	double n_pts = 0;
	for( size_t i = 0; i < eval_parts.size(); i++ )
	{
		p_overlap_.push_back(probs[i]);

		n_pts += eval_parts[i]->points.size();
		prob += probs[i] * (double)eval_parts[i]->points.size();
	}

	prob /= n_pts;

/*
  for( size_t i = 0; i < probs.size(); i++ ){ 
    cout << probs[i] << endl;
    prob+= probs[i];
  }
  if( probs.size() > 0 )
    prob /= probs.size();*/
  cout << "overlapping is: " << prob << endl;
  if( eval_parts.size() == 0 )
    prob = -1;
  return prob;
}


model vision::refine_models(vector<model> models,vector<set<int> > &parts)
{
  cout << "to refine models" << endl;
  model ref_m = decompose_object(models[0].cloud);
  extract_feature(ref_m);
  extract_feature_patch(ref_m);
  extract_feature_axis(ref_m);

  double min_sqr_dist;
  if( dims_meter_ )
    min_sqr_dist = 10.;
  else
    min_sqr_dist = 0.05;

  map<int,CloudPtr>::iterator it,it_2;

	map<int,int> surfs;
	map<int,int> surf_count;
	set<int> seen_surfs;

	for( size_t im = 0; im < models.size(); im++ )
	{
		model m = models[im];
		set<int> surf_vec;	
		map<int,int> surf_count_cur;
		for( it = m.surfaces_voxels.begin(); it != m.surfaces_voxels.end(); it++ )
		{
			CloudPtr cloud_ref = (*it).second;
			double min_dist = FLT_MAX;
			int min_id;
			int min_count;
			for( it_2 = ref_m.surfaces_voxels.begin(); it_2 != ref_m.surfaces_voxels.end(); it_2++ )
			{
				double cur_dist = 0;
				int count_local = 0;
				CloudPtr cloud_cur = (*it_2).second;
				search::KdTree<PointType>::Ptr tree (new search::KdTree<PointType>);
				tree->setInputCloud(cloud_cur);
				vector< int > k_indices; vector< float > k_sqr_distances;
				int k = 2;
				for( size_t i = 0;i < cloud_ref->points.size(); i++ )
				{
					tree->nearestKSearch(cloud_ref->points[i],k,k_indices,k_sqr_distances);
					if( k_sqr_distances[0] <= min_sqr_dist ){
						cur_dist += k_sqr_distances[0];
						count_local++;
					}
				}
				if( count_local > 0 )
					cur_dist /= (double)count_local;
				if( ( cur_dist < min_dist ) && ( count_local > 0 ) )
				{
					min_dist = cur_dist;
					min_id = (*it_2).first;
					min_count = count_local;
				}
			}
			surf_vec.insert(min_id);
			surf_count_cur[min_id] = min_count; 
		}
		cout << "in refine: " << surf_vec.size() << endl;
		set<int>::iterator its;
		for( its = surf_vec.begin(); its != surf_vec.end(); its++ )
		{
			if( seen_surfs.find(*its) != seen_surfs.end() )
			{
				double count = surf_count[*its];
				if( count > surf_count_cur[*its] )
					surfs[*its] = im;				
			}
			else
			{
				surfs[*its] = im;
				surf_count[*its] = surf_count_cur[*its];
			}
			seen_surfs.insert(*its);
		}
	}

	parts = vector<set<int> >(models.size());
	map<int,int>::iterator it_s;
	for( it_s = surfs.begin(); it_s != surfs.end(); it_s++ )
	{
		parts[(*it_s).second].insert((*it_s).first);	
		cout << "part " << (*it_s).first << " assigned to model : " << (*it_s).second << endl;
	}
  cout << "refinement done" << endl;
  return ref_m;		
}

model vision::refine_models(model m)
{
  cout << "to refine models" << endl;
  model ref_m = decompose_object(m.cloud);
  extract_feature(ref_m);
  extract_feature_patch(ref_m);
  extract_feature_axis(ref_m);

  double min_sqr_dist;
  if( dims_meter_ )
    min_sqr_dist = 10.;
  else
    min_sqr_dist = 0.05;

  map<int,CloudPtr>::iterator it,it_2;
  set<int> surf_vec;
  cout << "#of surface voxels: " << m.surfaces_voxels.size() << endl;
  for( it = m.surfaces_voxels.begin(); it != m.surfaces_voxels.end(); it++ )
  {
    CloudPtr cloud_ref = (*it).second;
    double min_dist = FLT_MAX;
    int min_id;
    for( it_2 = ref_m.surfaces_voxels.begin(); it_2 != ref_m.surfaces_voxels.end(); it_2++ )
    {
      double cur_dist = 0;
      int count_local = 0;
      CloudPtr cloud_cur = (*it_2).second;
      search::KdTree<PointType>::Ptr tree (new search::KdTree<PointType>);
      tree->setInputCloud(cloud_cur);
      vector< int > k_indices; vector< float > k_sqr_distances; 
      int k = 2;
      for( size_t i = 0;i < cloud_ref->points.size(); i++ )
      {
        tree->nearestKSearch(cloud_ref->points[i],k,k_indices,k_sqr_distances);
        if( k_sqr_distances[0] <= min_sqr_dist ){
          cur_dist += k_sqr_distances[0];
          count_local++;
        }
      }
      if( count_local > 0 )
        cur_dist /= (double)count_local;
      if( ( cur_dist < min_dist ) && ( count_local > 0 ) )
      {
        min_dist = cur_dist;
        min_id = (*it_2).first;
      }
    }
    surf_vec.insert(min_id);
  } 
 
  cout << "in refine: " << surf_vec.size() << endl;
  set<int>::iterator its;
  for( its = surf_vec.begin(); its != surf_vec.end(); its++ )
  {
    double cur_dist = 0;
    int count_p = 0;
    double epsilon = 1e-10;
    for( size_t i = 0; i < ref_m.surface_patch_normal[*its].size(); i++ )
    {
      int ref_id = ref_m.surface_voxel_map[*its][i];
      for( it = ref_m.surfaces_voxels.begin(); it != ref_m.surfaces_voxels.end(); it++ )
      {
        if( ref_m.surfaces_nns[*its].find((*it).first) == ref_m.surfaces_nns[*its].end() )
          continue;
        for( size_t j = 0; j < ref_m.surface_patch_axis[(*it).first].size(); j++ )
        {
          int cur_id = ref_m.surface_voxel_map[(*it).first][j];
          if( ref_m.segment_nns[ref_id].find(cur_id) == ref_m.segment_nns[ref_id].end() )
            continue;
          count_p ++;
          cout << "to get distance:" << endl;
          cur_dist += shape_context_distance(ref_m.surface_patch_axis[(*its)][i],ref_m.surface_patch_axis[(*it).first][j]);
        }
      }
    }
    if( count_p > 0 )
      cur_dist /= count_p;
    cout << "nns:  " << count_p << endl;
    if( ( count_p > 0 ) && ( cur_dist > epsilon ) )
      pair_dists_patch_.push_back(cur_dist);
  }
  cout << "refinement done to update model" << endl;
  model ref_m_tmp = ref_m;
	ref_m_tmp.id = m.id;
  for( it = ref_m.surfaces_voxels.begin(); it != ref_m.surfaces_voxels.end(); it++ )
  {
    cout << "refine voxel: "<< (*it).first << " : "<< ref_m.surfaces_voxels.size() << endl; 
    if( surf_vec.find((*it).first) != surf_vec.end() )
      continue;
    ref_m_tmp.surfaces_voxels.erase((*it).first);
    ref_m_tmp.surfaces.erase((*it).first);
    ref_m_tmp.surfaces_nns.erase((*it).first);
    ref_m_tmp.surface_patch_normal.erase((*it).first);
    ref_m_tmp.surface_patch_axis.erase((*it).first);
  }
  cout << "refinement done" << endl;
  return ref_m_tmp;
}

void vision::estimate_co_occurance(vector<model> models,vector<model> pro_models,vector<Hcluster> clusters)
{
  cout << "in estimate co-occurance" << endl;
  cout << "clusters size: " << clusters.size() << endl;
  occ_mat_= randu<Mat<double> >(clusters.size(),clusters.size());
  occ_mat_.fill(0);
    
  int n_samples = 0;
  for( size_t i = 0; i < pro_models.size(); i++ )
  {
    model m = pro_models[i];
    map<int,set<int> >::iterator it,it_2,it_3;
    Mat<int> seen_mat;
    int max_val = 0;
    for( it = m.surfaces.begin(); it != m.surfaces.end(); it++ )
    {
      for( size_t ip = 0; ip < m.surface_patch_axis[(*it).first].size(); ip++ )
      {
        int id = m.surface_voxel_map[(*it).first][ip];
        if( id > max_val )
          max_val = id;
      }
    }
    seen_mat = randu<Mat<int> >(max_val+1,max_val+1);
    seen_mat.fill(0);

    for( it = m.surfaces.begin(); it != m.surfaces.end(); it++ )
    {
      for( size_t ip = 0; ip < m.surface_patch_axis[(*it).first].size(); ip++ )
      {
        int id = m.surface_voxel_map[(*it).first][ip];
        Cube<double> patch_ref = m.surface_patch_axis[(*it).first][ip];
        vector<int> cls_ref = measure_similarity_codebook(clusters,patch_ref);
        for( it_2 = m.surfaces.begin(); it_2 != m.surfaces.end(); it_2++ )
        { 
          for( size_t j = 0; j < m.surface_patch_axis[(*it_2).first].size(); j++ )
          {
            int cur_id = m.surface_voxel_map[(*it_2).first][j];
            Cube<double> patch_cur = m.surface_patch_axis[(*it_2).first][j];
            vector<int> cls_cur = measure_similarity_codebook(clusters,patch_cur);
            if( m.segment_nns[id].find(cur_id) == m.segment_nns[id].end() )
              continue;
           // n_samples++;
            cout << "match: " << cls_ref.size() << ": " << cls_cur.size() << endl;
            Mat<int> local_mat(occ_mat_.n_rows,occ_mat_.n_cols); 
            local_mat.fill(0);
						if( ( cls_ref.size() > 0 ) && ( cls_cur.size() > 0 ) )
							n_samples++;
            for( size_t ic_1 = 0; ic_1 < cls_ref.size(); ic_1++ )
            {
              for( size_t ic_2 = 0;ic_2 < cls_cur.size(); ic_2++ )
              {
                local_mat(cls_ref[ic_1],cls_cur[ic_2]) ++;
                local_mat(cls_cur[ic_2],cls_ref[ic_1]) ++;
                if( ( local_mat(cls_ref[ic_1],cls_cur[ic_2]) > 1 ) || ( local_mat(cls_cur[ic_2],cls_ref[ic_1]) > 1 ) )
                  continue;
              //  if( cls_ref[ic_1] < cls_cur[ic_2] )
                  occ_mat_(cls_ref[ic_1],cls_cur[ic_2]) ++;
                //else
                  occ_mat_(cls_cur[ic_2],cls_ref[ic_1]) ++;
              }
            }

          }
        } 
      }
    }
  }
  cout << "#of samples: " << n_samples << endl;

 // double s = accu(occ_mat_);
  occ_mat_ /= n_samples;
  for( int i = 0; i < clusters.size(); i++ )
  {
    for( int j = 0; j < clusters.size();j++ )
    {
      double val = occ_mat_(i,j);
      if( occ_mat_(j,i) > val )
        val = occ_mat_(j,i);
       occ_mat_(i,j) = val;
       occ_mat_(j,i) = val;
    }
  }
  //occ_mat_ = occ_mat_tmp;
  cout << occ_mat_ << endl;
}

void vision::make_code_book(vector<Hcluster> &clusters)
{
	Mat<double> dist_cls(clusters.size(),1);

  string file_name; 
  if( !parts_cls_ )
    file_name = default_path_ + "cls-patch-thresh.txt";
  else
    file_name = default_path_ + "cls-parts-thresh.txt";

  string line;  
  ifstream ifs(file_name.c_str());
  if( ifs.is_open() )
    getline(ifs,line);
  ifs.close();
  stringstream ss(line);
  ss >> min_dist_cls_;

  double epsilon = 1e-5; 
  for( size_t i = 0; i < clusters.size(); i++ )
  {
    Cube<double> center(clusters[i].memebers[0].n_rows,clusters[i].memebers[0].n_cols,clusters[i].memebers[0].n_slices);
    center.fill(0);
    for( size_t j = 0; j < clusters[i].memebers.size(); j++ ){
     bool is_valid = true;
     for( int ic = 0; ic < clusters[i].memebers[j].n_elem; ic++ )
     {
       if( isnan(clusters[i].memebers[j](ic)) || isinf(clusters[i].memebers[j](ic)) )
         is_valid = false;
     }
     if( !is_valid )
       continue;
     center += clusters[i].memebers[j];
    }
    center /= clusters[i].memebers.size();
    clusters[i].center = center;
    
    double max_dist = -0.01;  
    for( size_t j = 0; j < clusters[i].memebers.size(); j++ )
    {
      double dist;
      if( parts_cls_ )
        dist = chi_hist_distance(clusters[i].memebers[j],center);
      else
        dist = shape_context_distance(clusters[i].memebers[j],center);
      if( dist > max_dist )
        max_dist = dist;
    } 
    clusters[i].dist_threshold = max_dist;  
  }
  for( size_t i = 0; i < clusters.size(); i++ )
  {
		dist_cls(i,0) = clusters[i].dist_threshold;
//		if( !parts_cls_ )
	//		clusters[i].dist_threshold = min_dist_cls_;
		/*clusters[i].dist_threshold = 0; */
		if( clusters[i].dist_threshold < min_dist_cls_ )
		//if(( clusters[i].dist_threshold < min_dist_cls_ ) && ( clusters[i].memebers.size() > 1 ))
      clusters[i].dist_threshold = min_dist_cls_;
		
		
		//clusters[i].dist_threshold = min_dist_cls_;
  }
  dist_cls.save("cluster-thresh.mat", arma_ascii);	
}


double vision::euc_hist_distance(Cube<double> hist1,Cube<double> hist2)
{
  double dist = 0;
  Cube<double> hist(hist1.n_rows,hist1.n_cols,hist1.n_slices);
  hist = hist1 - hist2;
  for( int i = 0; i < hist.n_elem; i++ ){ 
    dist += pow(hist(i,0,0),2);
  }
  dist = sqrt(dist);
  return dist;
}

double vision::inter_hist_distance(Cube<double> hist1,Cube<double> hist2)
{
  double dist = 0;
  for( int i = 0; i < hist1.n_elem; i++ )
  {
    if( hist1(i) < hist2(i) )
      dist += hist1(i);
    else
      dist += hist2(i);
  }
  double acc1 = accu(hist1);
  double acc2 = accu(hist2);
  dist = 0.5 * ( (dist/acc1) + (dist/acc2) );
  return dist;
}

double vision::chi_hist_distance(Cube<double> hist1,Cube<double> hist2)
{
  double dist = 0; 
  double epsilon = 1e-5;
  for( int i = 0; i < hist1.n_rows; i++ )
  {
    /*if( isnan(fabs(hist1(i,0,0))) || isinf(fabs(hist1(i,0,0))) || isnan(fabs(hist2(i,0,0))) || isinf(fabs(hist2(i,0,0))) )
    {
      dist = -1;
      break;
    }*/
    double cur_dist = pow(hist1(i,0,0) - hist2(i,0,0),2) / (hist1(i,0,0) + hist2(i,0,0));
    if( isnan(fabs(cur_dist)) || isinf(fabs(cur_dist)) )
      cur_dist = 0;
    dist += cur_dist;
  }
/*  if( isnan(fabs(dist)) || isinf(fabs(dist)) )
    dist = 0;*/
  return dist;
}

double vision::bha_hist_distance(Cube<double> hist1,Cube<double> hist2)
{
  double dist = 0;
  for( int i = 0; i < hist1.n_elem; i++ )
  {
    double cur_dist = sqrt(hist1(i) * hist2(i));
    dist += cur_dist;
  }
  double n1 = accu(hist1); double n2 = accu(hist2);
  n1 /= hist1.n_elem; n2 /= hist1.n_elem;
  double bias = (1./sqrt(n1*n2*pow(hist1.n_elem,2)));
 // cout << "bias: " << bias << " dist: " << dist << endl;
  dist = bias * dist;
 // cout << dist << endl;
  double epsilon = 1e-05;
  if( fabs(1. - dist) <= epsilon )
    return 0; 
  
  dist = sqrt(1. - dist);
  return dist;
}

double vision::shape_context_distance(Cube<double> hist1,Cube<double> hist2)
{
  double dist = 0;
  
  double min_dist1 = FLT_MAX;
  for( int i = 0; i < hist1.n_rows; i++ )
  {
    Cube<double> hist(hist1.n_rows,hist1.n_cols,hist1.n_slices);
    hist = hist2;
    for( int j = 0; j < hist.n_rows-i; j++ )
    {
      double tmp = hist(j+i,0,0);
      hist(j+i,0,0) = hist(j,0,0);
      hist(j,0,0) = tmp;    
    }
    double dist_cur = euc_hist_distance(hist1,hist);
    if( dist_cur < min_dist1 )
      min_dist1 = dist_cur;
  } 

  double min_dist2 = FLT_MAX;
  for( int i = 0; i < hist1.n_rows; i++ )
  {
    Cube<double> hist(hist1.n_rows,hist1.n_cols,hist1.n_slices);
    hist = hist1;
    for( int j = 0; j < hist.n_rows-i; j++ )
    {      
      double tmp = hist(j+i,0,0);
      hist(j+i,0,0) = hist(j,0,0);
      hist(j,0,0) = tmp;
    }
    double dist_cur = euc_hist_distance(hist1,hist);
    if( dist_cur < min_dist2 )
      min_dist2 = dist_cur;
  }
  dist = min_dist1 + min_dist2;
  return dist;
}


void vision::read_input()
{
  TiXmlDocument doc("../config-data.xml");
  bool loadOkay = doc.LoadFile();
  TiXmlHandle hdoc(&doc);
  if( loadOkay )
  { 
    cout << "succesfully loaded" << endl;
    TiXmlElement* elem = hdoc.FirstChild("config").FirstChild("dims").ToElement();
    if( elem ){
      cout << "elem dims found" << endl;
      string val_s = elem->GetText();
      stringstream ss(val_s);
      int val;
      ss >> val;
      cout << val << endl;
      exec_mode_.dims = val;
    }
    else
      cout << "elem dims not found" << endl;
    TiXmlElement* elem1 = hdoc.FirstChild("config").FirstChild("patch-cluster").ToElement();
    if( elem1 )
    {
      string val = elem1->GetText();
      cout << "patch-cluster: " << val << endl;
      exec_mode_.patch_cls_path = val;
    }

    TiXmlElement* elem2 = hdoc.FirstChild("config").FirstChild("part-cluster").ToElement();
    if( elem2 )
    {
      string val = elem2->GetText();
      cout << "part-cluster: " << val << endl;
      exec_mode_.part_cls_path = val;
    }
   
    TiXmlElement* elem3 = hdoc.FirstChild("config").FirstChild("mode").ToElement();
    if( elem3 )
    {
      string val = elem3->GetText();
      cout << "mode:" << val << endl;
      if( mode_map_.find(val) != mode_map_.end() ){
        exec_mode_.mode = mode_map_[val];
        cout << "mode map is: " << mode_map_[val] << endl;
      }
    }
 
    TiXmlElement* elem4 = hdoc.FirstChild("config").FirstChild("prototypes").ToElement();
    if( elem4 )
    {
      string val = elem4->GetText();
      cout << "prototypes:" << val << "." << endl;
      exec_mode_.pro_path = val;
    }

    TiXmlElement* elem5 = hdoc.FirstChild("config").FirstChild("part-mats").ToElement();
    if( elem5 )
    {
      string val = elem5->GetText();
      cout << "part-mats: " << val << endl;
      exec_mode_.part_mats_path = val;
    }

    TiXmlElement* elem6 = hdoc.FirstChild("config").FirstChild("part-cluster-mat").ToElement();
    if( elem6 )
    {
      string val = elem6->GetText();
      cout << "part-cluster-mat: " << val << endl;
      exec_mode_.part_cls_mat_path = val;
    }

    TiXmlElement* elem7 = hdoc.FirstChild("config").FirstChild("part-object-occ-mat").ToElement();
    if( elem7 )
    {
      string val = elem7->GetText();
      cout << "part-object-occ-mat: " << val << endl;
      exec_mode_.part_object_occ_mat_path = val;
    }

    TiXmlElement* elem8 = hdoc.FirstChild("config").FirstChild("patch-cluster-mat").ToElement();
    if( elem8 )
    {
      string val = elem8->GetText();
      cout << "patch-cluster-mat: " << val << endl;
      exec_mode_.patch_cls_mat_path = val;
    }

    TiXmlElement* elem9 = hdoc.FirstChild("config").FirstChild("eval-path").ToElement();
    if( elem9 )
    {
      string val = elem9->GetText();
      cout << "eval-path: " << val << endl;
      exec_mode_.eval_path = val;
    }

    TiXmlElement* elem10 = hdoc.FirstChild("config").FirstChild("eval-object-class").ToElement();
    if( elem10 )
    {
      string val = elem10->GetText();
      cout << "eval-object-class: " << val << endl;
      exec_mode_.eval_object_class_path = val;
    }

    TiXmlElement* elem11 = hdoc.FirstChild("config").FirstChild("input-pcd").ToElement();
    if( elem11 )
    {
      string val = elem11->GetText();
      cout << "input-pcd: " << val << endl;
      exec_mode_.input_pcd = val;
    }

    TiXmlElement* elem12 = hdoc.FirstChild("config").FirstChild("patch-pcd").ToElement();
    if( elem12 )
    {
      string val = elem12->GetText();
      cout << "patch-pcd: " << val << endl;
      exec_mode_.patch_pcd = val;
    }

    TiXmlElement* elem13 = hdoc.FirstChild("config").FirstChild("patches-path").ToElement();
    if( elem13 )
    {
      string val = elem13->GetText();
      cout << "patches-path: " << val << endl;
      exec_mode_.patches_path = val;
    }

    TiXmlElement* elem14 = hdoc.FirstChild("config").FirstChild("input-dir").ToElement();
    if( elem14 )
    {
      string val = elem14->GetText();
      cout << "input-dir: " << val << endl;
      exec_mode_.input_dir = val;
    }

    TiXmlElement* elem15 = hdoc.FirstChild("config").FirstChild("train-object-class").ToElement();
    if( elem15 )
    {
      string val = elem15->GetText();
      cout << "input-dir: " << val << endl;
      exec_mode_.train_objects_class = val;
    }

    TiXmlElement* elem16 = hdoc.FirstChild("config").FirstChild("default-path").ToElement();
    if( elem16 )
    {
      string val = elem16->GetText();
      cout << "default-path: " << val << endl;
      default_path_ = val;
		}

    TiXmlElement* elem17 = hdoc.FirstChild("config").FirstChild("voxel-resolution").ToElement();
    if( elem17 ){
      string val_s = elem17->GetText();
      stringstream ss(val_s);
      double val;
      ss >> val;
      voxel_resolution = val;
    }

    TiXmlElement* elem18 = hdoc.FirstChild("config").FirstChild("seed-resolution").ToElement();
    if( elem18 ){
      string val_s = elem18->GetText();
      stringstream ss(val_s);
      double val;
      ss >> val;
      seed_resolution = val;
    }

    TiXmlElement* elem19 = hdoc.FirstChild("config").FirstChild("patch-n-mat").ToElement();
    if( elem19 ){
      string val_s = elem19->GetText();
      exec_mode_.patch_n_cls_mat = val_s;
    }
    TiXmlElement* elem20 = hdoc.FirstChild("config").FirstChild("debug-mode").ToElement();
    if( elem20 ){
      string val_s = elem20->GetText();
      stringstream ss(val_s);
      bool val;
      ss >> val;
      debug_ = val;
    }

  }
  else
  {
    cout << "load not succesful" << endl;
  }
}

void vision::execution()
{
  mode_map_["sample"] = SAMPLE;
  mode_map_["load"] = LOAD;
  mode_map_["parts"] = PARTS;  
  mode_map_["parts_all"] = PARTS_ALL;
  mode_map_["cluster"] = CLUSTER;
  mode_map_["evaluate"] = EVALUATE;
	mode_map_["part_seg"] = PART_SEG;
  mode_map_["extract_patch"] = EXTRACT_PATCH;
  mode_map_["extract_patch_pts"] = EXTRACT_PATCH_PTS;
	mode_map_["estimate_patch_threshold"] = ESTIMATE_PATCH_THRESHOLD;
	mode_map_["estimate_part_threshold"] = ESTIMATE_PART_THRESHOLD;

  map<string,Mode>::iterator it;
  for( it = mode_map_.begin(); it != mode_map_.end(); it++ )
    cout << (*it).first << ":" << (*it).second << endl;

	exec_mode_.mode = INVALID;
  read_input();
   
  cout << "mode is: " << exec_mode_.mode << endl;
  switch( exec_mode_.mode )
  {
    case SAMPLE:
      sample_input_data();
			break;
    case LOAD:
      load_data();
			break;
    case PARTS:
      get_parts();
			break;
    case PARTS_ALL:
      get_all_parts();
			break;
    case CLUSTER:
      do_clustering();
			break;
    case EVALUATE:
      do_evaluation();
			break;
    case PART_SEG:
      do_parts_segmentation();
      break;
    case EXTRACT_PATCH:
      do_patch_extraction();
			break;
    case EXTRACT_PATCH_PTS:
      do_patch_extraction_pts();
			break;
		case ESTIMATE_PATCH_THRESHOLD:
			estimate_threshold_protoypes();
			break;
		case ESTIMATE_PART_THRESHOLD:
			estimate_part_threshold_protoypes();
			break;
    default:
      cout << "unknown operation" << endl;
  }
}


void vision::sample_input_data()
{
  string dir_name = exec_mode_.input_dir;
  dims_meter_ = exec_mode_.dims;
  vector<string> files;
  if( dir_name.find(".pcd") == string::npos ){
    files = get_all_files(dir_name);
    files = sample_data(files,0.7);
  }
}

void vision::load_data()
{
  dims_meter_ = exec_mode_.dims;
  load_cloud(exec_mode_.input_pcd);
}

void vision::get_parts()
{
  string dir_name = exec_mode_.input_dir;
  dims_meter_ = exec_mode_.dims;
  vector<string> files;
  if( dir_name.find(".pcd") == string::npos ){
    files = get_all_files(dir_name);
  }
  else
    files.push_back(dir_name);
  vector<model> models;
  for( size_t i = 0; i < files.size(); i++ )
  {
    cout << "file " << i << " : " << files.size() << endl;
    CloudPtr cloud = load_cloud(files[i]);
    model m = decompose_object(cloud);
    extract_feature(m);
    extract_feature_patch(m);
    extract_feature_axis(m);
    if( save_ )
      save_feature(m);
    m.id = file_name_;
    models.push_back(m);
  } 
}

void vision::get_all_parts()
{
  patch_ = false;
  string dir_name = exec_mode_.input_dir;
  dims_meter_ = exec_mode_.dims;
  vector<string> files;
  if( dir_name.find(".pcd") == string::npos ){
    files = get_all_files(dir_name);
  }
  else
    files.push_back(dir_name);
  vector<model> models;
  for( size_t i = 0; i < files.size(); i++ )
  {
    cout << "file " << i << " : " << files.size() << endl;
    CloudPtr cloud = load_cloud(files[i]);
    model m = decompose_object(cloud);
    extract_feature(m);
    extract_feature_patch(m);
    extract_feature_axis(m);
    if( save_ )
      save_feature(m);
    m.id = file_name_;
    models.push_back(m);
  }
}

void vision::estimate_threshold_protoypes()
{
  dims_meter_ = exec_mode_.dims;

  string pro_path = exec_mode_.pro_path;
  cout << "read prototype" << ":" << pro_path <<endl;
	vector<model> pro_models = read_prototypes(pro_path);	
	pair_dists_patch_.clear();
  cout << "refine prototype" << endl;
  for( size_t ip = 0; ip < pro_models.size(); ip++ ){
    cout << "to read" << pro_models[ip].id << endl;
    cout << ip << " from " << pro_models.size() << endl;
    pro_models[ip] = refine_models(pro_models[ip]);
  }

  string file_name;
  file_name = default_path_ + "cls-patch-thresh.txt";
  ofstream ofs(file_name.c_str());
	ofs << pair_dists_patch_[0] << endl;
  ofs.close();
}

void vision::do_clustering()
{
//	agg_max_ = true;
  string dir_name = exec_mode_.input_dir;
  dims_meter_ = exec_mode_.dims;

  string pro_path = exec_mode_.pro_path;
  cout << "read prototype" << ":" << pro_path <<endl;
  /*vector<model> pro_models = read_prototypes(pro_path);
*/
  string file_name;
  file_name = default_path_ + "cls-patch-thresh.txt";

	double val;
  string line;
  ifstream ifs(file_name.c_str());
  if( ifs.is_open() )
    getline(ifs,line);
  ifs.close();
  stringstream ss(line);
  ss >> val;
	pair_dists_patch_.push_back(val);

/*
	string models_path = "/media/TOSHIBA\ EXT/learned-models-bmvc-washington/";
	vector<model> models = read_prototypes(models_path,false);
	cout << "#of models: " << models.size() << endl;*/
  vector<string> files;
  if( dir_name.find(".pcd") == string::npos ){
    files = get_all_files(dir_name);
  }
  else
    files.push_back(dir_name);
  vector<model> models;
  for( size_t i = 0; i < files.size(); i++ )
  {
		cout << "to decompose: " << files[i] << endl;
		cout << i << " from " << files.size() << endl;
    CloudPtr cloud(new Cloud);
		cloud = load_cloud(files[i]);
		if( cloud->points.size() == 0 )
			continue;
    model m = decompose_object(cloud);
    extract_feature(m);
    extract_feature_patch(m);
    extract_feature_axis(m);
    if( save_ )
      save_feature(m);
    m.id = file_name_;
    models.push_back(m);
  }
  cout << "to cluster" << endl;

  /*pair_dists_patch_.clear();
  cout << "refine prototype" << endl;
  for( size_t ip = 0; ip < pro_models.size(); ip++ ){
    cout << "to read" << pro_models[ip].id << endl;
		cout << ip << " from " << pro_models.size() << endl;
    pro_models[ip] = refine_models(pro_models[ip]);
  } 
*/
  cout << "start cluster: " << models.size() << endl;
  clock_t tf;
  tf = clock();
	
  vector<Hcluster> clusters = cluster_features(models);
	tf = clock() - tf;
  float tf_f = ((float)tf)/CLOCKS_PER_SEC;
  cout << "clustering time : " << tf_f << endl; 	
}


void vision::do_parts_segmentation()
{
  string patch_cls = exec_mode_.patch_cls_mat_path;
  Mat<double> mat_occ;
  mat_occ.load(patch_cls);
  occ_mat_ = mat_occ;
  occ_mat_.save("patch-mat.mat", arma_ascii);
 // for( int r = 0; r < occ_mat_.n_rows;r++)
//    occ_mat_(r,r) = 1.;

  cout << "to load patch nn" << endl;
  cout << "load: " << exec_mode_.patch_n_cls_mat << endl;
  string patch_n_cls = exec_mode_.patch_n_cls_mat;
  patch_occ_n_.load(patch_n_cls);
  cout << "after load" << endl;
  patch_occ_n_.save("patch-cluster-nn.mat", arma_ascii);

  for( int r = 0; r < patch_occ_n_.n_rows;r++)
    patch_occ_n_(r,r) = 0.;

  string dir_name = exec_mode_.patch_cls_path;
  dims_meter_ = exec_mode_.dims;
  parts_cls_ = false;
  vector<string> dirs = get_all_files(dir_name,"cls");
  vector<model> models;
  vector<Hcluster> clusters = read_clusters_patch(dirs,models);
  make_code_book(clusters);
  cout << "patch clusters size: " << clusters.size() << endl;
  cout << "to read prototypes" << endl;
  string pro_path = exec_mode_.pro_path;
  cout << "to evaluate" << endl;
  string eval_path = exec_mode_.eval_path;
  vector<model> eval_models = make_evaluation_models(eval_path);
  double overlap_all = 0;
  int count_p = 0;

  string path_r = default_path_ + "region-thresh.txt";
  ifstream ifs(path_r.c_str());
  if( ifs.is_open() )
  {
    string str;
    if( ifs.good() )
    {
      getline(ifs,str);
      stringstream ss(str);
      ss >> region_threshold_;
    }
  }


  ofstream ofs("overlap-file.txt");

	map<string,int> match_model;
  cout << "after make evaluation models" << endl;
  for( size_t i = 0; i < eval_models.size(); i++ ){
    cout << "to evaluate model patch init" << endl;
    vector<set<int> > regs = recognize_parts(eval_models[i],clusters);
    vector<vector<set<int> > > parts_h = parts_h_;
    vector<vector<set<int> > > parts_h_pts = parts_h_pts_;
    make_parts_visualization(parts_h,parts_h_pts,eval_models[i]);
    parts_h_.clear();
    parts_h_pts_.clear();

    vector<CloudPtr> regs_clouds = extract_part_pts(eval_models[i],regs);
    double overlap = estimate_overlap(regs_clouds,eval_models[i]);
		for( size_t ip = 0; ip < p_overlap_.size(); ip++ )
		{
			ofs << eval_models[i].id << " " << p_ids_[ip] << " " << p_overlap_[ip] << endl;
		}
		ofs << eval_models[i].id << " "<< overlap << endl;
    if( overlap < 0 )
      continue;
    count_p++;
    overlap_all += overlap;
  }
  overlap_all /= (double) count_p;
  cout << "number of objs: " << count_p << endl;
  cout << "overall overlap is: " << overlap_all << endl;

	ofs.close();
}

vector<set<int> > vision::recognize_parts(model m,vector<Hcluster> clusters)
{
	cout << "in recognize parts" << endl;
	regs_all_.clear();
  vector<set<int> > regs = evaluate_model_patch(m,clusters);
	cout << "after init" << endl;
  parts_h_.push_back(regs);
  vector<set<int> > regs_cur = regs;
  int count_iter = 0;
  double epsilon = 1e-5;
	regs_all_[count_iter] = regs;
	regs_potential_[count_iter] = potential_;
  epsilon = min_potential_;
//	epsilon = 0.5;
  if( potential_ > epsilon ){
    do{
      count_iter ++;
      cout << "iter: " << count_iter << endl;
      regs = regs_cur;
      regs_cur = evaluate_model_patch_new(m,clusters,regs);
      region_id_.clear();
      region_id_pre_.clear();
      parts_h_pts_.push_back(regs);
			regs_all_[count_iter] = regs_cur;
			regs_potential_[count_iter] = potential_;
    }while( regs_cur.size() < regs.size() );
  }
  else
    parts_h_pts_.push_back(regs);
  return regs;
}


void vision::do_evaluation()
{
  string dir_name = exec_mode_.input_dir;
  dims_meter_ = exec_mode_.dims;
  vector<string> test_data = get_all_files(dir_name);
  map<string,vector<string> > test_map;
  set<string> file_set;
  for( size_t i = 0; i < test_data.size(); i++ )
  {
		cout << "test data: " << test_data[i] << endl;
    string root = test_data[i].substr(test_data[i].find_last_of("/")+1);
    root = root.substr(0,root.find(".pcd"));
    string cue = "_out";
    string sub_root = root.substr(root.find(cue)+cue.length());
    root = root.substr(0,root.find(cue));
    if( sub_root.length() == 0 )
      continue;
    test_map[root].push_back(test_data[i]);
    file_set.insert(root);
  }
  map<int,string> map_id;
  vector<vector<CloudPtr> >clouds;
  map<string,vector<string> >::iterator it;
  for( it = test_map.begin(); it != test_map.end(); it++ )
  {
    vector<CloudPtr> cur_clouds;
    vector<string> vec = (*it).second;
    for( size_t i = 0; i < vec.size(); i++ ){
      CloudPtr cloud = load_cloud(vec[i]);
      cur_clouds.push_back(cloud);
    }
    clouds.push_back(cur_clouds);
    map_id[clouds.size()-1] = (*it).first;
  }
	ofstream ofs("overlap-file.txt");
  cout << "to overlap" << endl;
  double overlap_all = 0;
  double epsilon = 1e-5;
  for( size_t j = 0; j < clouds.size(); j++ )
  {
    model m;
    m.id = map_id[j];
    cout << "pattern: " << map_id[j] << endl;
    double overlap = estimate_overlap(clouds[j],m);
    for( size_t ip = 0; ip < p_overlap_.size(); ip++ )
    {
			ofs << m.id << " " << p_ids_[ip] << " " << p_overlap_[ip] << endl; 
    }
    ofs << m.id << " "<< overlap << endl;
		cout << "obj overlap done" << endl;
    if( overlap > epsilon )
      overlap_all += overlap;
  }
  overlap_all /= (double)clouds.size();
  cout << "overall overlap is: " << overlap_all << endl;
	ofs.close();
}

void vision::estimate_part_threshold_protoypes()
{
  string dir_name = exec_mode_.patch_cls_path;
  dims_meter_ = exec_mode_.dims;
  cout << "to read clusters" << endl;
  vector<string> dirs = get_all_files(dir_name,"cls");
  vector<model> models;
  vector<Hcluster> clusters = read_clusters_patch(dirs,models);
  make_code_book(clusters);

  cout << "to read prototypes" << endl;
  string pro_path = exec_mode_.pro_path;
  vector<model> pro_models = read_prototypes(pro_path);
  cout << "#of prototypes" << pro_models.size() << endl;

  pair_dists_patch_.clear();

	pair_dists_.clear();
  for( size_t ip = 0; ip < pro_models.size(); ip++ ){
    cout << "id before refine: " << pro_models[ip].id << endl;
		cout << ip << " from : " << pro_models.size() << endl;
    pro_models[ip] = refine_models(pro_models[ip]);
  }


	cout << "to get co-occurance" << endl;
  estimate_co_occurance(models,pro_models,clusters);
  string dir_patch_cls = default_path_ + "patch-cluster/";
  boost::filesystem::path dir_pcls(dir_patch_cls);
  boost::filesystem::create_directories(dir_pcls);
  stringstream ss;
  ss << default_path_ << "patch-cluster/" << "patch-occ";
  occ_mat_.save(ss.str());
}

void vision::do_patch_extraction()
{
  dims_meter_ = exec_mode_.dims;
  CloudPtr cloud = load_cloud(exec_mode_.input_pcd);
  extract_patch(cloud,exec_mode_.patch_pcd);
}


void vision::do_patch_extraction_pts()
{
  dims_meter_ = exec_mode_.dims;
  string dir_name = exec_mode_.input_dir;
	string orig_dir = exec_mode_.input_pcd;

	vector<string> files;
	if(orig_dir.find(".pcd") != string::npos)
		files.push_back(orig_dir);
	else
		files = get_all_files(orig_dir,".pcd");

	for( size_t i = 0; i < files.size(); i++ )
	{
		cout << "orig file: " << files[i] << endl;
		string root = files[i].substr(files[i].find_last_of("/")+1);
		root = root.substr(0,root.find_last_of("."));
		root = dir_name + root;
		cout << "root to open: " << root << endl;
		vector<string> orig_pts = get_all_files(root,"");
		if( orig_pts.size() == 0 )
			continue;
		if( orig_pts[0].length() <= 2 )
			continue;
		CloudPtr cloud = load_cloud(files[i]);
		string path = orig_pts[0];
		path = path.substr(0,path.find("."));
		path = path + "/";
		cout << "patch path: " << path << endl;
		vector<string> files_cur = get_all_files(path,".pcd");
		vector<CloudPtr> clouds;
		for(size_t j = 0; j < files_cur.size(); j++)
		{
			cout << "patch file: " << files_cur[j] << endl;
			CloudPtr cloud_patch = load_cloud(files_cur[j]);
			clouds.push_back(cloud_patch);
		}
	
		model m;

		vector<set<int> > surfs_vecs = refine_multiple_patches(clouds,cloud,m);
		cout << "to save prototypes" << endl;
		for( size_t i = 0; i < surfs_vecs.size(); i++ ){
			stringstream ss;
			ss << m.id << "_" << i;
			file_name_ = ss.str();
			save_prototype(m,surfs_vecs[i]);
		}
	
	}

}

