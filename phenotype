							float yechang = 0;
							for (int i = 1; i < yzhong->points.size(); i++)
							{
								float yechangf = 0;
								yechangf = sqrt(pow((yzhong->points[i - 1].x - yzhong->points[i].x), 2) + pow((yzhong->points[i - 1].y - yzhong->points[i].y), 2) +
									pow((yzhong->points[i - 1].z - yzhong->points[i].z), 2));
								yechang = yechang + yechangf;

							}
							cout << "叶长：" << yechang * 100 << endl;
							if (out1.is_open())
							{
								out1 << yechang * 100 << "	";
								out1 << ss0.str() << "\n";
							}
							else
							{
								cout << "error" << endl;
							}
