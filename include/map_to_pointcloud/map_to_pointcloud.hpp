/**
* @file     map_to_pointcloud.hpp
* @brief    地図から点群データを生成するクラスMapToPointCloudDataの定義ヘッダファイル
* @author   S.Kumada
* @date     2023/09/04
* @details  2D占有格子地図から、点群データを生成するためのクラスの定義ヘッダファイル
*/

#ifndef MAP_TO_POINTCLOUD_H_
#define MAP_TO_POINTCLOUD_H_

#include <ros/ros.h> // 基本ライブラリ
#include <nav_msgs/OccupancyGrid.h> // グリットマップ
#include <sensor_msgs/PointCloud2.h> // 配信するデータ型
#include <sensor_msgs/PointField.h>
#include <pcl_ros/point_cloud.h>

#include "utilities.h"

// 専有率定義
#define     FREESPACE_COST_GRIDMAP		0
#define     UNKNOWN_COST_GRIDMAP		-1

/**
 * @brief 点群データ生成クラス
 * @details OccupancyGrid型の地図からPointCloud形式の点群データを生成する
 */
class MapToPointCloudData
{
    public:

        /**
        * @brief   MapToPointCloudDataクラスのコンストラクタ
        * @details 初期化を行う
        */
        MapToPointCloudData(ros::NodeHandle node);

        /**
        * @brief   MapToPointCloudDataクラスのデストラクタ
        * @details オブジェクトの破棄等の終了処理を行う
        */
        ~MapToPointCloudData();

        /**
        * @brief        地図データ受信コールバック関数
        * @param[in]    nav_msgs::OccupancyGridPtr msg 地図データ
        * @return       void
        * @details      点群データに変換する地図を受信する
        */
        void recvMapCB(const nav_msgs::OccupancyGridPtr msg);

        /**
        * @brief        地図座標からワールド座標への変換関数
        * @param[in]    (unsigned int) map_x X軸のセル座標
        * @param[in]    (unsigned int) map_y Y軸のセル座標
        * @param[in]    (unsigned int) world_x X軸のワールド座標
        * @param[in]    (unsigned int) world_y Y軸のワールド座標
        * @return       void
        * @details      地図座標からワールド座標に変換する
        */
       inline void mapToWorld(unsigned int map_x, unsigned int map_y, double& world_x, double& world_y) const
       {
            world_x = origin_x_ + (map_x + 0.5) * resolution_;
            world_y = origin_y_ + (map_y + 0.5) * resolution_;
       }

        /**
        * @brief        地図データのインデックスの索引関数
        * @param[in]    (unsigned int) mx セルのX座標
        * @param[in]    (unsigned int) my セルのY座標
        * @return       unsigned int インデックス値
        * @details      セル座標からインデックスを索引し、結果を返す
        */
        inline unsigned int getIndex(unsigned int mx, unsigned int my) const
        {
            return my * map_cell_width_ + mx;
        }

        /**
        * @brief        地図データのインデックス値を座標に変換する
        * @param[in]    (unsigned int) インデックス値
        * @param[in]    (unsigned int) mx セルのX座標
        * @param[in]    (unsigned int) my セルのY座標
        * @return       void
        * @details      地図座標からワールド座標に変換する
        */
        inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
        {
            my = index / map_cell_width_;
            mx = index - (my * map_cell_width_);
        }

    private:
		ros::Publisher pub_cloud; // 変換後の点群データのパブリッシャ
		ros::Subscriber sub_map; // 変換する地図のサブスクライバ

        double occupied_thresh_; // 完全専有として判断する専有確率（0〜100％）のしきい値
        std::string global_frame_; // 地図のフレーム名
        unsigned int map_cell_width_; // 地図の幅[cell]
        unsigned int map_cell_height_; // 地図の高さ[cell]
        int publish_times_; // 配信回数
		double origin_x_; // 原点のX座標
		double origin_y_; // 原点のY座標
		double resolution_; // 解像度
		double sensor_height_; // 点群センサーの擬似的な高さ

};

#endif