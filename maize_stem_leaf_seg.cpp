				size_t j = 1110, k = 0;

				int yepian2 = 0,yepian3=0;
				yepian3 = yepianbz;
				float xiaxian = 0;


				yepian2 = int(xinxi[yepianbz][0]);
				for (; yepianbz < cv.size(); yepianbz++)
				{
					cout << cv[int(xinxi[yepianbz][0])]->points.size() << endl;
					pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
					*cloud_cluster = *cv[int(xinxi[yepianbz][0])];
				
					std::stringstream  sss, s;
					sss << i << "oushi" << j;
					s << i << "yumi-3" << j << ".pcd";
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloud_cluster, a[j % 14][0], a[j % 14][1], a[j % 14][2]);
					viewer->addPointCloud<pcl::PointXYZI>(cloud_cluster, single_color, sss.str());

					j++;
					
					pcl::PointXYZI mint0, maxt0, centert0, cenye1;
					pcl::PointXYZI mint1, maxt1, centert1;
					pcl::PointXYZI juxingk;
					
					//						writer.write<pcl::PointXYZI>("D:/pcl-2022/shuju/oushi/di8ou/" + s.str(), *cloud_cluster, true);
					
					pcl::PointCloud<pcl::PointXYZI>::Ptr cloudt0(new pcl::PointCloud<pcl::PointXYZI>), cloudt1(new pcl::PointCloud<pcl::PointXYZI>);
					pcl::PointIndices::Ptr inliers0(new pcl::PointIndices());
					pcl::PointIndices::Ptr inliers1(new pcl::PointIndices());

					for (size_t i = 0; i < cloud_cluster->points.size(); i++)
						if ((cloud_cluster->points[i].x < maxd0.x + Fb1 && cloud_cluster->points[i].x >= mind0.x - Fb1) && (cloud_cluster->points[i].y < maxd0.y + Fb1 && cloud_cluster->points[i].y >= mind0.y - Fb1))//会向后吃一个分辨率
						{
							inliers0->indices.push_back(i);
						}

					if (inliers0->indices.size() > 0)
					{
						pcl::ExtractIndices<pcl::PointXYZI> extract0;
						extract0.setInputCloud(cloud_cluster);
						extract0.setIndices(inliers0);
						extract0.setNegative(false);
						extract0.filter(*cloudt0);
						extract0.setNegative(true);
						extract0.filter(*cloudt1);
						inliers0->indices.clear();
						
						pcl::getMinMax3D(*cloudt0, mint0, maxt0);
						centert0.x = (mint0.x + maxt0.x) / 2;
						centert0.y = (mint0.y + maxt0.y) / 2;
						centert0.z = (mint0.z + maxt0.z) / 2;

						juxingk.x = (maxt0.x - mint0.x) / 2;
						juxingk.y = (maxt0.y - mint0.y) / 2;
						juxingk.z = (maxt0.z - mint0.z) / 2;
						
						
					}

					for (size_t i = 0; i < cloudt1->points.size(); i++)
						if ((cloudt1->points[i].x < maxd0.x + (2 * Fb1) && cloudt1->points[i].x >= mind0.x - (2 * Fb1)) && (cloudt1->points[i].y < maxd0.y + (2 * Fb1) && cloudt1->points[i].y >= mind0.y - (2 * Fb1)))//会向后吃一个分辨率
						{
							inliers0->indices.push_back(i);
						}

					if (inliers0->indices.size() > 0)
					{
						pcl::ExtractIndices<pcl::PointXYZI> extract0;
						extract0.setInputCloud(cloudt1);
						extract0.setIndices(inliers0);
						extract0.setNegative(false);
						extract0.filter(*cloudt1);
						inliers0->indices.clear();

						pcl::getMinMax3D(*cloudt1, mint1, maxt1);
						centert1.x = (mint1.x + maxt1.x) / 2;
						centert1.y = (mint1.y + maxt1.y) / 2;
						centert1.z = (mint1.z + maxt1.z) / 2;

						
						Eigen::Vector4f vec1(centert0.x - centert1.x, centert0.y - centert1.y, centert0.z - centert1.z, 0);//方向向量
						
						double d2 = vec1.dot(vec1);
						double d = sqrt(d2);
					
						std::stringstream  sss, s;
						sss << i << "oushi" << j;
						s << i << "yumi-3" << j << ".pcd";
						pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloudt1, a[j % 14][0], a[j % 14][1], a[j % 14][2]);
						viewer->addPointCloud<pcl::PointXYZI>(cloudt1, single_color, sss.str());
						j++;

						
						for (int pan = 0; pan < 30; pan++)
						{
							pcl::PointXYZI cs;
							pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());

							centert1 = centert0;
							centert0.x = centert0.x + vec1.x() / 2;
							centert0.y = centert0.y + vec1.y() / 2;
							centert0.z = centert0.z + vec1.z() / 2;
							
							cs.x = centert0.x;
							cs.y = centert0.y;
							cs.z = centert0.z + (2 * juxingk.z);

							for (size_t i = 0; i < cloud->points.size(); i++)
								if ((cloud->points[i].x < centert0.x + juxingk.x && cloud->points[i].x >= centert0.x - juxingk.x) &&
									(cloud->points[i].y < centert0.y + juxingk.y && cloud->points[i].y >= centert0.y - juxingk.y) &&
									(cloud->points[i].z < centert0.z + juxingk.z && cloud->points[i].z >= centert0.z - juxingk.z)
									)//会向后吃一个分辨率
								{
									inliers1->indices.push_back(i);
								}
							cout << "shuliang1111111::::" << inliers1->indices.size() << endl;

							for (size_t i = 0; i < cloud->points.size(); i++)
								if ((cloud->points[i].x < cs.x + juxingk.x && cloud->points[i].x >= cs.x - juxingk.x) &&
									(cloud->points[i].y < cs.y + juxingk.y && cloud->points[i].y >= cs.y - juxingk.y) &&
									(cloud->points[i].z < cs.z + juxingk.z && cloud->points[i].z >= cs.z - juxingk.z)
									)//会向后吃一个分辨率
								{
									inliers2->indices.push_back(i);
								}
							cout << "shuliang222222::::" << inliers2->indices.size() << endl;


							//是否停止生长
							if (inliers1->indices.size() > 2)
								if (inliers2->indices.size() >(inliers1->indices.size()*0.5))//chuding  0.6
									pan = 30;
							inliers2->indices.clear();



							if (inliers1->indices.size() > 0 && pan < 30)
							{
								pcl::ExtractIndices<pcl::PointXYZI> extract0;
								extract0.setInputCloud(cloud);
								extract0.setIndices(inliers1);
								extract0.setNegative(false);
								extract0.filter(*cloudye1);
								extract0.setNegative(true);
								extract0.filter(*cloud);


								//点云合并。
								*cv[int(xinxi[yepianbz][0])] = *cv[int(xinxi[yepianbz][0])] + *cloudye1;
								inliers1->indices.clear();
								
								//修改生长方向。
								pcl::PointXYZI mintye1, maxtye1;
								pcl::getMinMax3D(*cloudye1, mintye1, maxtye1);
								cenye1.x = (mintye1.x + maxtye1.x) / 2;
								cenye1.y = (mintye1.y + maxtye1.y) / 2;
								cenye1.z = (mintye1.z + maxtye1.z) / 2;
								Eigen::Vector4f vec2(cenye1.x - centert1.x, cenye1.y - centert1.y, cenye1.z - centert1.z, 0);//方向向量
								
								double yd2 = vec2.dot(vec2);
								double yd = sqrt(yd2);
								
								vec1.x() = vec2.x() * d / yd;
								vec1.y() = vec2.y() * d / yd;
								vec1.z() = vec2.z() * d / yd;
								//设置顶层分割下限。
								if (yepian3 == yepianbz)
								{
													
									xiaxian = maxtye1.z;
								}


//								float angle;
//								angle = pcl::getAngle3D(vec2, v3, true);
//								cout << "->angle(角度) = " << angle << "°" << endl;
											
								std::stringstream  ss;
								ss << i << "oushi" << j;
								pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_colory1(cloudye1, a[j % 14][0], a[j % 14][1], a[j % 14][2]);
								viewer->addPointCloud<pcl::PointXYZI>(cloudye1, single_colory1, ss.str());
								j++;
							}
						}

						std::stringstream  ssyp;
						ssyp << "5yumi-3" << zhizhushu << "ye" << yepianbz<<".pcd";
						writer.write<pcl::PointXYZI>("D:/pcl-2023/shuju/zengzhang/" + ssyp.str(), *cv[int(xinxi[yepianbz][0])], true);

						int ksh = 1;

//						while (ksh == 1)
//						{
//							viewer->spinOnce(5000);
//							cin >> ksh;
//						}

					}
					
				
