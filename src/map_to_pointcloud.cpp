/**
* @file     map_to_pointcloud.cpp
* @brief    地図から点群データを生成するクラスMapToPointCloudDataの実装ソースファイル
* @author   S.Kumada
* @date     2023/09/04
* @details  2D占有格子地図から、点群データを生成を行う
*/

#include <map_to_pointcloud/map_to_pointcloud.hpp>


MapToPointCloudData::MapToPointCloudData(ros::NodeHandle node)
{
    sensor_msgs::PointField field;
    std::string sub_topic_name;
    std::string pub_topic_name;
    ros::NodeHandle param_node("~");

    // パラメータ読み込み
    getParam(param_node, "map_topic_name",   sub_topic_name,    std::string("/map"));   // 点群化する地図の取得トピック名
    getParam(param_node, "cloud_topic_name", pub_topic_name,    sub_topic_name + std::string("_cloud"));    // 点群化後の配信トピック名
    getParam(param_node, "occupied_thresh",  occupied_thresh_,  50.0);  // 占有してると判断する確立のしきい値
    getParam(param_node, "global_frame",     global_frame_,     std::string("lidar")); // 変換後の点群データのフレーム名
    getParam(param_node, "sensor_height",    sensor_height_,    0.35); // 点群センサーの擬似的な高さ
    getParam(param_node, "publish_times",    publish_times_,    0); // 配信回数

    // サブスクライバ定義
    sub_map = node.subscribe(sub_topic_name,  ROS_QUEUE_SIZE_1, &MapToPointCloudData::recvMapCB, this);
    
    // パブリッシャ定義
    pub_cloud = node.advertise<pcl::PointCloud<pcl::PointXYZ> >(pub_topic_name.c_str(), 10, true);

}


MapToPointCloudData::~MapToPointCloudData()
{
    // 何もしない
}

void MapToPointCloudData::recvMapCB(const nav_msgs::OccupancyGridPtr msg)
{
    unsigned int map_size           = msg->data.size();   // 地図のサイズ
    std::vector<int8_t> map_data    = msg->data;    // 地図データ
    map_cell_width_ = msg->info.width;
    map_cell_height_    = msg->info.height;
    origin_x_   = msg->info.origin.position.x;
    origin_y_   = msg->info.origin.position.y;
    resolution_ = msg->info.resolution;


    // PointCloudデータの作成
    pcl::PointCloud<pcl::PointXYZ> cloud_data;

    // 地図データをクロールして閾値以上のセル座標を抜き出す
    for(unsigned int idx = 0; idx < map_size; ++idx)
    {
        // 閾値判定
        if(	map_data[idx] == UNKNOWN_COST_GRIDMAP   ||  // UNKNOWN	 = -1
            map_data[idx] == FREESPACE_COST_GRIDMAP ||  // FREESPACE = 0
            map_data[idx] < occupied_thresh_ )  		// しきい値となる専有確率以下
        {
            continue;
        }
        
        unsigned int cell_x, cell_y; // 該当セルの座標値
        double x, y;    // 原点からの距離
        
        indexToCells(idx, cell_x, cell_y);  // セル座標を求める
        mapToWorld(cell_x, cell_y, x, y);   // 地図座標系からワールド座標系に変換

        // データの格納
        pcl::PointXYZ point_data;
        point_data.x = (float)x;
        point_data.y = (float)y;
        point_data.z = sensor_height_;  // z座標はセンサの高さに固定
        
        cloud_data.points.push_back(point_data);
    }

    // 配信する点群データの作成
    auto pub_msg = cloud_data.makeShared(); // スマートポインタの取得

    // ヘッダ情報を積み込む
    pub_msg->header.frame_id = global_frame_; // フレーム名
    pcl_conversions::toPCL(ros::Time::now(), pub_msg->header.stamp); // 送信時間

#ifdef DEBUG
    ROS_INFO("!!!!!Pub PointCloud Data!!!!!");
#endif
    // 配信処理
    ros::Rate sleep_rate = 10; // 配信周期Hz
    int pub_time = publish_times_; // 配信回数

    for(; pub_time > 0; --pub_time)
    { // 配信回数分ループ
        sleep_rate.sleep();
        pub_cloud.publish(pub_msg);
    }

}