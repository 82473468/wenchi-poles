//
// Created by wenchi on 17-6-19.
//

#include "extraction.h"
namespace robosense
{
    extraction::extraction(ros::NodeHandle node, ros::NodeHandle private_nh)
    {

        std::string topic = private_nh.param("topic", std::string("rslidar_points"));
        sub_tracklets = node.subscribe(topic, 10, &extraction::gridCallback, (extraction *) this);

        pub_poles = node.advertise<sensor_msgs::PointCloud2>("/rslidar_points_global", 10);
        pub_poles_position = node.advertise<sensor_msgs::PointCloud2>("/rs_features", 10);

        in_clip_min_height = private_nh.param("in_clip_min_height", float(-1.3));
        in_clip_max_height = private_nh.param("in_clip_max_height", float(-0.5));

        in_clip_min_height_2 = private_nh.param("in_clip_min_height", float(0.5));
        in_clip_max_height_2 = private_nh.param("in_clip_max_height", float(1.2));

        gWidth = private_nh.param("gWidth", float(50.));
        gHeight = private_nh.param("gHeight", float(50.));
        miniGrid = private_nh.param("miniGrid", float(0.15));

        dilation_size = 1;

        //计算栅格长宽
        gridH = gHeight / miniGrid;
        gridW = gWidth / miniGrid;

        //用于分割的栅格图
        grid = cv::Mat::zeros(gridH, gridW, CV_8UC1);
        grid_2 = cv::Mat::zeros(gridH, gridW, CV_8UC1);
        grid_3 = cv::Mat::zeros(gridH, gridW, CV_8UC1);

        temp_1 = cv::Mat::zeros(gridH, gridW, CV_8UC1);
        temp_2 = cv::Mat::zeros(gridH, gridW, CV_8UC1);

        colorsNum = 200;

        generateColors(_colors, colorsNum);

        frame_num = 1;

    }

    void extraction::publishpoles_tree(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud)
    {
        sensor_msgs::PointCloud2 output_poles;
        pcl::toROSMsg(*poles_cloud, output_poles);
        output_poles.header.frame_id = "base_link2";
        in_publisher->publish(output_poles);
    }

    void extraction::gridCallback(const sensor_msgs::PointCloud2 &msg)
    {
        robosense_header = msg.header;
        double tmp_time = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9;
        std::cout.precision(20);
//    std::cout<<"the time is "<<tmp_time<<std::endl;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pole_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

        float theta = -(94.0/180)*3.1415926;
        transform_1(0, 0) = cosf(theta);
        transform_1(0, 1) = -sinf(theta);
        transform_1(1, 0) = sinf(theta);
        transform_1(1, 1) = cosf(theta);
        pcl::transformPointCloud(*cloud, *cloud_2, transform_1);


        publishpoles_tree(&pub_poles, cloud_2);
        clock_t start = clock();
        pushFrame(cloud_2);
        clock_t end = clock();

    }

    void extraction::pushFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud_ptr_2(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointCloud<pcl::PointXYZI>::Ptr poles_position(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        //利用栅格法进行两个高度分割
        clipCloud(cloud, clipped_cloud_ptr, in_clip_min_height, in_clip_max_height);

        clipCloud(cloud, clipped_cloud_ptr_2, in_clip_min_height_2, in_clip_max_height_2);

        genGrid(clipped_cloud_ptr, grid);

        genGrid(clipped_cloud_ptr_2, grid_2);

        dilation_size = 1;
        cv::Mat element = getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                cv::Point(dilation_size, dilation_size));
        dilate(grid, grid, element);

        dilate(grid_2, grid_2, element);
//
        cv::Mat label;//int型
        icvprCcaBySeedFill(grid, label);

        cv::Mat label_2;
        icvprCcaBySeedFill(grid_2, label_2);

        //将柱状分割提取出来
        extractTree(label, temp_1);
        extractTree(label_2, temp_2);

        //将两个高度融合一起
        matormat(temp_1, temp_2, grid_3);

        cv::Mat label_3;
        icvprCcaBySeedFill(grid_3, label_3);
        //进一步判断，将柱状物体分割出来
        std::vector<poleslabel> poles;

        genClusters2(label_3, cloud, poles);

        for(int i=0;i<poles.size();++i)
        {
            pcl::PointXYZI temp_poles;
            temp_poles.x=poles[i].center.x;
            temp_poles.y=poles[i].center.y;
            temp_poles.z=poles[i].center.z;
            poles_position->points.push_back(temp_poles);
        }

        pcl::PointXYZI position_point;
        position_point.x = -10000;
        position_point.y = -10000;
        position_point.z = 0;
        poles_position->points.push_back(position_point);


        publishpoles(&pub_poles_position, poles_position);

    }


    void extraction::clusterstoFrames(const std::vector<pcl::PointCloud<pcl::PointXYZI> > in_cluster,
                                           std::vector<poleslabel2> &curPoles,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr poles_position)
    {
        curPoles.clear();

        std::vector<pcl::PointCloud<pcl::PointXYZI> >::const_iterator it = in_cluster.begin();
        int tmp_label = 0;

        for(; it != in_cluster.end(); ++it)
        {
            float x_position = 0;
            float y_position = 0;
            std::vector<cv::Point2f> points;

            for(int i = 0; i < (*it).points.size(); ++i)
            {
                x_position += (*it).points[i].x;
                y_position += (*it).points[i].y;
                points.push_back(cv::Point2f((*it).points[i].x, (*it).points[i].y));
            }

            x_position /= (*it).points.size();
            y_position /= (*it).points.size();
            cv::RotatedRect rect = cv::minAreaRect(points);

            float long_size = rect.size.height > rect.size.width ? rect.size.height : rect.size.width;
            float short_size = rect.size.height > rect.size.width ? rect.size.width : rect.size.height;
            float len = sqrtf(x_position * x_position + y_position * y_position);
            float offset = (long_size - short_size) / 2.0;
            x_position = x_position + offset * x_position / len;
            x_position = x_position + offset * x_position / len;

            poleslabel2 pole_tmp;
            pole_tmp.cloud = *it;
            pole_tmp.label = tmp_label++;
            pole_tmp.location = cv::Point2f(x_position, y_position);
            curPoles.push_back(pole_tmp);

            pcl::PointXYZI position_point;
            position_point.x = x_position;
            position_point.y = y_position;
            position_point.z = 0;
            poles_position->points.push_back(position_point);

        }
    }


    bool compX(cv::Point2f pt1, cv::Point2f pt2)
    {
        return pt1.x < pt2.x;
    }


    void extraction::extractTree(const cv::Mat label, cv::Mat &mat)
    {
        double min, max;
        minMaxIdx(label, &min, &max);
        std::vector<int> labelnum;
        labelnum.resize(max + 1, 0);
        mat.setTo(0);
        for(int i = 0; i < label.rows; ++i)
        {
            for(int j = 0; j < label.cols; ++j)
            {
                labelnum[label.at<int>(i, j)]++;
            }
        }

        for(int i = 0; i < label.rows; ++i) {
            for(int j = 0; j < label.cols; ++j) {
                if (labelnum[label.at<int>(i, j)] < 30)
                    mat.at<uchar>(i, j) = 1;
            }
        }

    }


    void extraction::euclideanDistance(std::vector<poleslabel> &poles,
                                            std::vector<pcl::PointCloud<pcl::PointXYZI> > &in_cluster)
    {
        std::vector<poleslabel>::iterator it = poles.begin();
        for(; it != poles.end() - 1; ++it)
        {
            std::vector<poleslabel>::iterator it_son = it + 1;
            for(; it_son != poles.end(); ++it_son)
            {
                if (it == it_son) continue;
                if ((*it).label == 0) continue;
                if ((*it_son).label == 0) continue;
                float distance = sqrtf(
                        pow(((*it).center.x - (*it_son).center.x), 2) + pow(((*it).center.y - (*it_son).center.y), 2));
                if (distance < 2)
                {
                    (*it).label = 0;
                    (*it_son).label = 0;
                }
            }
        }
        std::vector<poleslabel>::iterator it_son2 = poles.begin();
        for(; it_son2 != poles.end(); ++it_son2)
        {
            if ((*it_son2).label == 1)
            {
                in_cluster.push_back((*it_son2).cloud);
            }
        }
    }


    void extraction::genClusters2(const cv::Mat label,
                                       const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                       std::vector<poleslabel> &poles)
    {
        double min, max;
        minMaxIdx(label, &min, &max);
        std::vector<pcl::PointCloud<pcl::PointXYZI> > tmpPCDs;
        tmpPCDs.resize(max);

        std::vector<int> labelnum;
        labelnum.resize(max + 1, 0);
        for(int i = 0; i < label.rows; ++i)
        {
            for(int j = 0; j < label.cols; ++j)
            {
                labelnum[label.at<int>(i, j)]++;
            }
        }

        for(int i = 0; i < in_cloud_ptr->size(); ++i)
        {
            pcl::PointXYZI tmpI = in_cloud_ptr->points[i];
            cv::Point2i tmpIndex = trans(cv::Point2f(tmpI.x, tmpI.y));
            if (tmpIndex.x >= 0 && tmpIndex.x < gridW && tmpIndex.y >= 0 && tmpIndex.y < gridH)
            {
                int index = label.at<int>(tmpIndex.y, tmpIndex.x);
                if (index <= 0)continue;
                if (labelnum[index] > 50) continue;
                tmpPCDs[index - 1].push_back(tmpI);
            }
        }

        sensor_msgs::PointCloud2 output;
        pcl::PointCloud<pcl::PointXYZI>::Ptr poles_posi(new pcl::PointCloud<pcl::PointXYZI>);
        poles_posi->clear();
        sensor_msgs::PointCloud2 output_poles;
        for(int i = 0; i < tmpPCDs.size(); ++i)
        {
            if (tmpPCDs[i].points.size() < 10) continue;
            float z_min = 999, z_max = -999;
            float x_pos = 0, y_pos = 0, z_pos = 0;
            std::vector<cv::Point2f> points;
            points.clear();
            for(int j = 0; j < tmpPCDs[i].points.size(); ++j)
            {
                if (tmpPCDs[i].points[j].z < z_min)
                    z_min = tmpPCDs[i].points[j].z;
                if (tmpPCDs[i].points[j].z > z_max)
                    z_max = tmpPCDs[i].points[j].z;
                x_pos += tmpPCDs[i].points[j].x;
                y_pos += tmpPCDs[i].points[j].y;
                z_pos += tmpPCDs[i].points[j].z;
                points.push_back(cv::Point2f(tmpPCDs[i].points[j].x, tmpPCDs[i].points[j].y));
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
            *cloud_ = tmpPCDs[i];
            x_pos /= tmpPCDs[i].points.size();
            y_pos /= tmpPCDs[i].points.size();
            z_pos /= tmpPCDs[i].points.size();

            //********利用判定来识别柱子
            bool ignore = false;
            float distance = sqrtf(x_pos * x_pos + y_pos * y_pos);
            if (distance < 8 && (z_max - z_min) < 2)
                ignore = true;
            if (distance > 8 && (z_max - z_min) < 2.4)
                ignore = true;
            if (ignore) continue;
            features(cloud_);
            if (countPoints > 1) continue;
//        std::cout<<"\033[32m the cov_scalar is \033[0m"<<cov_scalar<<"the scalar of points is:"<<scalarPoints<<std::endl;
            if (scalarPoints > 3) continue;
            if (cov_scalar > 3) continue;
            cv::RotatedRect rect = cv::minAreaRect(points);
            float long_size = rect.size.height > rect.size.width ? rect.size.height : rect.size.width;
            float short_size = rect.size.height > rect.size.width ? rect.size.width : rect.size.height;
//        std::cout<<"the long/short is "<<long_size/short_size<<std::endl;
            if (long_size > 0.4 && long_size / short_size > 2) continue;
//        std::cout << " the x and y position of poles is " << x_pos << "and " << y_pos << std::endl;
            poleslabel pole;
            pole.cloud = tmpPCDs[i];
            pole.center = cv::Point3f(x_pos, y_pos, z_pos);
            pole.label = 1;
            poles.push_back(pole);
        }
    }

    void extraction::matormat(const cv::Mat temp1, const cv::Mat temp2, cv::Mat &temp3)
    {
        temp3.setTo(0);
        for(int i = 0; i < temp1.rows; ++i)
        {
            for(int j = 0; j < temp1.cols; ++j)
            {
                if (temp1.at<uchar>(i, j) == 1 || temp2.at<uchar>(i, j) == 1)
                    temp3.at<uchar>(i, j) = 1;
            }
        }
    }


    void extraction::showcylinder(std::vector<cv::Point2f> centers)
    {

        visualization_msgs::MarkerArray markers;
        markers.markers.clear();

        std::vector<cv::Point2f>::iterator it_son = centers.begin();
        int idx = 0;
        for(; it_son != centers.end(); ++it_son)
        {
            visualization_msgs::Marker marker;
            marker.header = robosense_header;
            marker.ns = "basic_shapes";
            marker.id = idx;
            marker.type = visualization_msgs::Marker::CYLINDER;

            marker.pose.position.x = (*it_son).x;
            marker.pose.position.y = (*it_son).y;
            marker.pose.position.z = 2.5;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.25;
            marker.scale.y = 0.25;
            marker.scale.z = 8;

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5;

            marker.lifetime = ros::Duration(0.2);
            idx++;
            markers.markers.push_back(marker);
        }

        marker_pub.publish(markers);

    }


    void extraction::clipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                                    float in_min_height, float in_max_height)
    {
        out_cloud_ptr->points.clear();
        for(unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
        {
            if (in_cloud_ptr->points[i].z >= in_min_height &&
                in_cloud_ptr->points[i].z <= in_max_height)
            {
                out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
            }
        }
    }


    void extraction::genGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, cv::Mat &mat)
    {
        mat.setTo(0);
        for(int i = 0; i < in_cloud_ptr->size(); ++i)
        {
            pcl::PointXYZI tmpI = in_cloud_ptr->points[i];
            cv::Point2i tmpIdx = trans(cv::Point2f(tmpI.x, tmpI.y));

            if (tmpIdx.x >= 0 && tmpIdx.x < gridW && tmpIdx.y >= 0 && tmpIdx.y < gridH)
            {
                mat.at<uchar>(tmpIdx.y, tmpIdx.x) = 1;
            }
        }
    }

    cv::Point2i extraction::trans(cv::Point2f pt)
    {
        return cv::Point2i(int(pt.x / miniGrid) + gridW / 2, int(pt.y / miniGrid) + gridH / 2);
    }


    void extraction::icvprCcaBySeedFill(const cv::Mat &_binImg, cv::Mat &_lableImg)
    {

        if (_binImg.empty() || _binImg.type() != CV_8UC1)
        {
            return;
        }

        cv::Mat edge_binImg = cv::Mat::zeros(_binImg.rows + 2, _binImg.cols + 2, CV_8UC1);

        _binImg.copyTo(edge_binImg(cv::Rect(1, 1, _binImg.cols, _binImg.rows)));

        _lableImg.release();
        edge_binImg.convertTo(_lableImg, CV_32SC1);

        int label = 1;  // start by 2

        int rows = _binImg.rows - 1;
        int cols = _binImg.cols - 1;

        for(int i = 1; i < rows - 1; i++)
        {
            int *data = _lableImg.ptr<int>(i);
            for(int j = 1; j < cols - 1; j++)
            {
                if (data[j] == 1)
                {
                    std::stack<std::pair<int, int> > neighborPixels;
                    neighborPixels.push(std::pair<int, int>(i, j));// pixel position: <i,j>


                    ++label;  // begin with a new label
                    while (!neighborPixels.empty())
                    {
                        // get the top pixel on the stack and label it with the same label
                        std::pair<int, int> curPixel = neighborPixels.top();
                        int curX = curPixel.first;
                        int curY = curPixel.second;
                        _lableImg.at<int>(curX, curY) = label;

                        // pop the top pixel
                        neighborPixels.pop();

                        // push the 4-neighbors (foreground pixels)
                        if (_lableImg.at<int>(curX, curY - 1) == 1)
                        {// left pixel
                            neighborPixels.push(std::pair<int, int>(curX, curY - 1));
                        }
                        if (_lableImg.at<int>(curX, curY + 1) == 1)
                        {// right pixel
                            neighborPixels.push(std::pair<int, int>(curX, curY + 1));
                        }
                        if (_lableImg.at<int>(curX - 1, curY) == 1)
                        {// up pixel
                            neighborPixels.push(std::pair<int, int>(curX - 1, curY));
                        }
                        if (_lableImg.at<int>(curX + 1, curY) == 1)
                        {// down pixel
                            neighborPixels.push(std::pair<int, int>(curX + 1, curY));
                        }

                    }
                }
            }
        }


        cv::Mat tmp = _lableImg.clone();
        _lableImg.release();
        _lableImg = cv::Mat::zeros(tmp.rows - 2, tmp.cols - 2, CV_32SC1);
        tmp(cv::Rect(1, 1, _lableImg.cols, _lableImg.rows)).copyTo(_lableImg);
        tmp.release();

    }


    void extraction::features(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)
    {

        //点数
        Pointnum.resize(1, 0);
        Pointnum[0] = cloud_->points.size();
        //协方差
        Cov_mat.resize(6, 0);
        float x_avr = 0;
        float y_avr = 0;
        float z_avr = 0;

        for(int pp = 0; pp < cloud_->points.size(); ++pp)
        {
            x_avr += cloud_->points[pp].x;
            y_avr += cloud_->points[pp].y;
            z_avr += cloud_->points[pp].z;
        }
        x_avr /= cloud_->points.size();
        y_avr /= cloud_->points.size();
        z_avr /= cloud_->points.size();

        for(int pp = 0; pp < cloud_->points.size(); ++pp)
        {
            Cov_mat[0] += (cloud_->points[pp].x - x_avr) * (cloud_->points[pp].x - x_avr);//cov(x,x)
            Cov_mat[1] += (cloud_->points[pp].y - y_avr) * (cloud_->points[pp].y - y_avr);//cov(y,y)
//            Cov_mat[2] += (cloud_->points[pp].x - x_avr) * (cloud_->points[pp].y - y_avr);//cov(x,y)
//            Cov_mat[3] += (cloud_->points[pp].x - x_avr) * (cloud_->points[pp].z - z_avr);//cov(x,z)
//            Cov_mat[4] += (cloud_->points[pp].y - y_avr) * (cloud_->points[pp].z - z_avr);//cov(y,z)
//            Cov_mat[5] += (cloud_->points[pp].z - z_avr) * (cloud_->points[pp].z - z_avr);//cov(z,z)
        }

        for(int i = 0; i < 6; ++i)
        {
            Cov_mat[i] /= (cloud_->points.size() - 1);
        }

        float cov_min = 999, cov_max = -999;
        for(int i = 0; i < 2; ++i)
        {
            if (Cov_mat[i] < cov_min)
                cov_min = Cov_mat[i];
            if (Cov_mat[i] > cov_max)
                cov_max = Cov_mat[i];
        }
        cov_scalar = cov_max / cov_min;
        //切片
        float min_z = 100;
        float max_z = -100;
        for(int i = 0; i < cloud_->points.size(); i++)
        {
            if (cloud_->points[i].z < min_z)
                min_z = cloud_->points[i].z;
            if (cloud_->points[i].z > max_z)
                max_z = cloud_->points[i].z;
        }

        int sliceNum = 7;
        Slice_mat.resize(sliceNum * 2, 0);
        float sliceStep = (max_z - min_z) / sliceNum;

        countPoints = 0;
        countPoints_top = 0;
        countPoints_bottom = 0;
        scalarPoints = 0;
        if (sliceStep > 0.1)
        {
            std::vector<std::vector<pcl::PointXYZI> > sliceHistgram;
            sliceHistgram.resize(sliceNum);
            for(int i = 0; i < cloud_->points.size(); i++)
            {
                int sliceIndex = (cloud_->points[i].z - min_z) / sliceStep;
                if (sliceIndex == sliceNum)
                    sliceIndex -= 1;
                sliceHistgram[sliceIndex].push_back(cloud_->points[i]);
            }

            for(int i = 0; i < sliceHistgram.size(); i++)
            {
                if (sliceHistgram[i].size() == 0)
                {
                    countPoints++;
                }

                if (i < 3)
                    countPoints_bottom += sliceHistgram[i].size();
                else
                    countPoints_top += sliceHistgram[i].size();
            }
            scalarPoints = countPoints_top / countPoints_bottom;
        }

        feature.clear();
        feature.push_back(Pointnum[0]);
        feature.push_back((max_z - min_z));
        feature.push_back(countPoints);
    }

    void extraction::publishpoles(const ros::Publisher *in_publisher,
                                       const pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud)
    {
        sensor_msgs::PointCloud2 output_poles;
        pcl::toROSMsg(*poles_cloud, output_poles);
        output_poles.header = robosense_header;
        in_publisher->publish(output_poles);
    }

}
