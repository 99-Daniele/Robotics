void ros::init(argv, argc, std::string node_name, uint32_t options); ros::init(argc, argv, "my_node_name");
ros::init(argc, argv, "my_node_name", ros::init_options::AnonymousName);
ros::NodeHandle nh;