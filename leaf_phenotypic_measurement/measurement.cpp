	for (std::vector<pcl::Vertices>::const_iterator it = triangles.polygons.begin(); it != triangles.polygons.end(); ++it)
	{
	
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_yep(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointNormal>::Ptr ct(new pcl::PointCloud<pcl::PointNormal>);
		for (std::vector<char32_t>::const_iterator pit = it->vertices.begin(); pit != it->vertices.end(); pit++)
			ct->points.push_back(mls_points_normal->points[*pit]);
	
		ct->width = ct->points.size();
		ct->height = 1;
		ct->is_dense = true;
		float ab, ac, bc, bp, mianji;
		ab = 0;
		ac = 0;
		bc = 0;
		bp = 0;
		mianji = 0;
	
		ab = sqrt(pow((ct->points[0].x - ct->points[1].x), 2) + pow((ct->points[0].y - ct->points[1].y), 2) + pow((ct->points[0].z - ct->points[1].z), 2));
		ac = sqrt(pow((ct->points[0].x - ct->points[2].x), 2) + pow((ct->points[0].y - ct->points[2].y), 2) + pow((ct->points[0].z - ct->points[2].z), 2));
		bc = sqrt(pow((ct->points[1].x - ct->points[2].x), 2) + pow((ct->points[1].y - ct->points[2].y), 2) + pow((ct->points[1].z - ct->points[2].z), 2));
		bp = (ab + ac + bc) / 2;
		mianji = sqrt(bp * (bp - ab) * (bp - ac) * (bp - bc));
		mianjisum = mianjisum + mianji;
	
	
	}
	cout << "面积1：" << mianjisum * 10000 << endl;
	if (out1.is_open())
	{
		out1 << mianjisum * 10000 << "	";
	}
	else
	{
		cout << "error" << endl;
	}
	
	//叶长
	{
	
		pcl::PointCloud<pcl::PointXYZ>::Ptr clouda0(new pcl::PointCloud<pcl::PointXYZ>());
		clouda0->points.resize(mls_points_normal->points.size());
		clouda0->width = mls_points_normal->points.size();
		clouda0->height = 1;
		clouda0->is_dense = true;
		for (size_t i = 0; i < mls_points_normal->points.size(); i++)
		{
			clouda0->points[i].x = mls_points_normal->points[i].x;
			clouda0->points[i].y = mls_points_normal->points[i].y;
			clouda0->points[i].z = mls_points_normal->points[i].z;
	
		}
		
		pcl::PointXYZ mind0, maxd0, center0;
		pcl::getMinMax3D(*cloud, mind0, maxd0);
		center0.x = (maxd0.x + mind0.x) / 2;
		center0.y = (maxd0.y + mind0.y) / 2;
		center0.z = (maxd0.z + mind0.z) / 2;
	//						cout << center0 << endl;
		pcl::PointXYZ mind1, maxd1;
		pcl::getMinMax3D(*cloud1, mind1, maxd1);
		float lz = 0, lxy = 0, lxz=0;
		lz = maxd1.z - mind1.z;
		lxy = sqrt(pow((maxd1.x - mind1.x), 2) + pow((maxd1.y - mind1.y), 2));
		lxz = sqrt(pow((maxd1.x - mind1.x), 2) + pow((maxd1.z - mind1.z), 2));
	
	
		pcl::PointCloud<pcl::PointXYZ>::Ptr yzhong(new pcl::PointCloud<pcl::PointXYZ>());
		vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> yeqingj;
		
		
		int sha = 0;
		if (lz < lxy)//另外还没写
		{
			
			cout << "beishumaxyyyyy" << lxy / Fb1 << endl;
			float yekuanmax = 0;
			pcl::PointCloud<pcl::PointXYZ>::Ptr c2(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr c3(new pcl::PointCloud<pcl::PointXYZ>);
			for (int beishu = 1; beishu < (lxy / Fb1 + 8); beishu++)
			{
				
	//								cout << "beishu" <<beishu<< endl;
				pcl::PointIndices::Ptr inliers0(new pcl::PointIndices());
				pcl::PointIndices::Ptr inliers1(new pcl::PointIndices());
				for (size_t i = 0; i < clouda0->points.size(); i++)
					if (sqrt(pow((clouda0->points[i].x - center0.x), 2) + pow((clouda0->points[i].y - center0.y), 2)) < beishu * Fb1)//会向后吃一个分辨率
					{
						inliers0->indices.push_back(i);
					}
				pcl::PointXYZ pm;
				
				if (inliers0->indices.size() > 0)
				{
					pcl::ExtractIndices<pcl::PointXYZ> extract0;
					extract0.setInputCloud(clouda0);
					extract0.setIndices(inliers0);
					extract0.setNegative(false);
					extract0.filter(*c2);
					extract0.setNegative(true);
					extract0.filter(*clouda0);
					inliers0->indices.clear();
	
				
					std::stringstream  sss, s;
					sss << i << "oushi" << beishu;
					s << i << "yumi-3" << beishu << ".pcd";
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(c2, a[beishu % 14][0], a[beishu % 14][1], a[beishu % 14][2]);
					viewer->addPointCloud<pcl::PointXYZ>(c2, single_color, sss.str());
	
				
	
					std::stringstream  ssy1;
	
					ssy1 << "8yumi-3" << i << "ye" << j <<"- "<<beishu<<".pcd";
					writer.write<pcl::PointXYZ>("D:/pcl-2023/shuju/mls/" + ssy1.str(), *c2);
	
	
					double mx = 0, my = 0, mz = 0;
					for (size_t i = 0; i < c2->points.size(); i++)
					{
						mx = mx + c2->points[i].x;
						my = my + c2->points[i].y;
						mz = mz + c2->points[i].z;
	
					}
					pm.x = mx / c2->points.size();
					pm.y = my / c2->points.size();
					pm.z = mz / c2->points.size();
					yzhong->points.push_back(pm);
					//叶宽计算
					
					float yekuan0 = 0;
	
	//									cout << c3->points.size() << endl;
					if (yzhong->points.size()>1)
					{
						if (c3->points.size() > 0)
						{
							pcl::PointXYZ fangx;
							fangx.x = yzhong->points[yzhong->points.size() - 1].x - yzhong->points[yzhong->points.size() - 2].x;
							fangx.y = yzhong->points[yzhong->points.size() - 1].y - yzhong->points[yzhong->points.size() - 2].y;
							fangx.z = yzhong->points[yzhong->points.size() - 1].z - yzhong->points[yzhong->points.size() - 2].z;
							
							Eigen::Vector4d vec1(fangx.x, fangx.y, fangx.z, 0);//方向向量
	//											cout << vec1[0] << endl;
							vec1.normalize();//单位化
							float dmax0 = 0, dmax1=0;
							if (vec1[0] > vec1[1])
							{
								float panduan = 0;
								panduan = yzhong->points[yzhong->points.size() - 2].y;
								for (int i = 0; i < c3->points.size(); i++)
								{
									if (c3->points[i].y>panduan)
									{
										Eigen::Vector4d vec2(c3->points[i].x - yzhong->points[yzhong->points.size() - 2].x, c3->points[i].y - yzhong->points[yzhong->points.size() - 2].y, c3->points[i].z - yzhong->points[yzhong->points.size() - 2].z, 0);// 空间点方向向量
										//								std::cout << "v2：" << vec2<< endl;
										double dst = vec1.cross3(vec2).squaredNorm();
										//								std::cout << "dst：" << dst << endl;
										double d = sqrt(dst);
										if (d > dmax0)
											dmax0 = d;
	
									}
									else
									{
										Eigen::Vector4d vec2(c3->points[i].x - yzhong->points[yzhong->points.size() - 2].x, c3->points[i].y - yzhong->points[yzhong->points.size() - 2].y, c3->points[i].z - yzhong->points[yzhong->points.size() - 2].z, 0);// 空间点方向向量
										//								std::cout << "v2：" << vec2<< endl;
										double dst = vec1.cross3(vec2).squaredNorm();
										//								std::cout << "dst：" << dst << endl;
										double d = sqrt(dst);
										if (d > dmax1)
											dmax1 = d;
									}
								}
								if (dmax0 != 0 && dmax1 != 0)
								{
									yekuan0 = dmax0 + dmax1;
	//													cout << "yekuanyyyyyyyy" << yekuan0 << endl;
								}
								else
								{
	//													cout << "shsssssss" << endl;
									yekuan0 = 2 * dmax0 + 2 * dmax1;
	//													cout << "yekuanyyyyyyyy" << yekuan0 << endl;
								}
	
							}
	
							else
							{
								float panduan = 0;
								panduan = yzhong->points[yzhong->points.size() - 2].x;
								for (int i = 0; i < c3->points.size(); i++)
								{
									if (c3->points[i].x>panduan)
									{
										Eigen::Vector4d vec2(c3->points[i].x - yzhong->points[yzhong->points.size() - 2].x, c3->points[i].y - yzhong->points[yzhong->points.size() - 2].y, c3->points[i].z - yzhong->points[yzhong->points.size() - 2].z, 0);// 空间点方向向量
										//								std::cout << "v2：" << vec2<< endl;
										double dst = vec1.cross3(vec2).squaredNorm();
										//								std::cout << "dst：" << dst << endl;
										double d = sqrt(dst);
										if (d > dmax0)
											dmax0 = d;
	
									}
									else
									{
										Eigen::Vector4d vec2(c3->points[i].x - yzhong->points[yzhong->points.size() - 2].x, c3->points[i].y - yzhong->points[yzhong->points.size() - 2].y, c3->points[i].z - yzhong->points[yzhong->points.size() - 2].z, 0);// 空间点方向向量
										//								std::cout << "v2：" << vec2<< endl;
										double dst = vec1.cross3(vec2).squaredNorm();
										//								std::cout << "dst：" << dst << endl;
										double d = sqrt(dst);
										if (d > dmax1)
											dmax1 = d;
									}
								}
								if (dmax0 != 0 && dmax1 != 0)
								{
									yekuan0 = dmax0 + dmax1;
	//													cout << "yekuanxxxxxx" << yekuan0 << endl;
								}
								else
								{
	//													cout << "shxxxxxxxxx" << endl;
									yekuan0 = 2 * dmax0 + 2 * dmax1;
	//													cout << "yekuanxxxxxx" << yekuan0 << endl;
								}
							}
	
						}
					}
	
	
					if (yekuanmax < yekuan0)
					{
						yekuanmax = yekuan0;
					}
					
	//									cout << "叶宽：" << yekuanmax << endl;
	
					*c3 = *c2;
					//叶倾角计算
					
					if (yeqingj.size() < 5)
					{
	//										cout << "yeqingj.size() " << yeqingj.size() << endl;
						yeqingj.push_back(c2);
						if (yeqingj.size() == 4)
						{
							Eigen::Vector4f vec2(yzhong->points[3].x - yzhong->points[2].x, yzhong->points[3].y - yzhong->points[2].y, yzhong->points[3].z - yzhong->points[2].z, 0);//方向向量
							float angle;
							angle = pcl::getAngle3D(vec2, v3, true);
							cout << "3叶倾角->angle(角度) = " << angle << "°" << endl;
							if (out1.is_open())
							{
								out1 << angle << "	";
							}
							else
							{
								cout << "error" << endl;
							}
						}
	
					}
					
					
	
				}
	
			}//	
	//							cout << "叶宽：" << yekuanmax << endl;
	
			//叶倾角计算
	//							cout << "yeqingj.size() " << yeqingj.size() << endl;
		
				if (yeqingj.size() == 3)
				{
					Eigen::Vector4f vec2(yzhong->points[2].x - yzhong->points[1].x, yzhong->points[2].y - yzhong->points[1].y, yzhong->points[2].z - yzhong->points[1].z, 0);//方向向量
					float angle;
					angle = pcl::getAngle3D(vec2, v3, true);
					cout << "2叶倾角->angle(角度) = " << angle << "°" << endl;
					if (out1.is_open())
					{
						out1 << angle << "	";
					}
					else
					{
						cout << "error" << endl;
					}
				}
				else if (yeqingj.size() == 2)
					{
						Eigen::Vector4f vec2(yzhong->points[1].x - yzhong->points[0].x, yzhong->points[1].y - yzhong->points[0].y, yzhong->points[1].z - yzhong->points[0].z, 0);//方向向量
						float angle;
						angle = pcl::getAngle3D(vec2, v3, true);
						cout << "2叶倾角->angle(角度) = " << angle << "°" << endl;
						if (out1.is_open())
						{
							out1 << angle << "	";
						}
						else
						{
							cout << "error" << endl;
						}
					}
				else if (yeqingj.size() == 1)
				{
					cout << "2叶倾角->angle(角度) =xxxxx "  << "°" << endl;
					if (out1.is_open())
					{
						out1 <<"xxxxxx" << "	";
					}
					else
					{
						cout << "error" << endl;
					}
				}
			cout << "叶宽：" << yekuanmax * 100 << endl;
			if (out1.is_open())
			{
				out1 << yekuanmax * 100 << "	";
			}
			else
			{
				cout << "error" << endl;
			}
	
			yzhong->width = yzhong->points.size();
			yzhong->height = 1;
			yzhong->is_dense = true;
			
	//							pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color196(yzhong, 0, 255, 0);
	//							viewer->addPointCloud<pcl::PointXYZ>(yzhong, single_color196, "d9d0");
	//							viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "d9d0");
		
			while (!viewer->wasStopped())
			{
				viewer->spinOnce(1000);
			}
	
			float yechang = 0;
			for (int i = 1; i < yzhong->points.size(); i++)
			{
				float yechangf = 0;
				yechangf = sqrt(pow((yzhong->points[i - 1].x - yzhong->points[i].x), 2) + pow((yzhong->points[i - 1].y - yzhong->points[i].y), 2) + 
						   pow((yzhong->points[i - 1].z - yzhong->points[i].z), 2));
				yechang = yechang + yechangf;
	
			}
			cout << "叶长：" << yechang << endl;
			if (out1.is_open())
			{
				out1 << yechang * 100 << "	";
				out1 << ss0.str() << "\n";
	
			}
			else
			{
				cout << "error" << endl;
			}
	
			
	
		}
	
	
