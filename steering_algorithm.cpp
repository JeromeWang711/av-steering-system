// Autonomous Vehicle Steering Algorithm
// Developed by Zhaoyu Wang as part of DIT637 course project
// University of Gothenburg, 2026
//
// Note: This file contains only the core algorithm implementation.
// The surrounding vehicle framework (shared memory, image capture, etc.) was provided by the course and is not included here.
// Framework: https://github.com/chalmers-revere/working-with-rec-files/tree/master/cpp-opencv
                
                // 1. Get current time for the output record
                cluon::data::TimeStamp sampleTimeStamp = sharedMemory->getTimeStamp().second;
                int64_t timeMicro = cluon::time::toMicroseconds(sampleTimeStamp);
                sharedMemory->unlock();

                // 2. Cut the image to only look at the road
                int startY = static_cast<int>(img.rows * 0.40); 
                int boxHeight = static_cast<int>(img.rows * 0.50); 
                cv::Rect roadArea(0, startY, img.cols, boxHeight);
                cv::Mat roadImage = img(roadArea);

                // 3. Change color to HSV to find cones easily
                cv::Mat hsvImage;
                cv::cvtColor(roadImage, hsvImage, cv::COLOR_BGR2HSV);

                cv::Mat blueMask;
                cv::Scalar blueMin(95, 90, 55);
                cv::Scalar blueMax(135, 255, 255);
                cv::inRange(hsvImage, blueMin, blueMax, blueMask);

                cv::Mat yellowMask;
                cv::Scalar yellowMin(15, 80, 55); 
                cv::Scalar yellowMax(45, 255, 255);
                cv::inRange(hsvImage, yellowMin, yellowMax, yellowMask);

                // 4. Fill small holes in the cones to make them solid blocks
                cv::Mat brush = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
                cv::morphologyEx(blueMask, blueMask, cv::MORPH_CLOSE, brush);
                cv::morphologyEx(yellowMask, yellowMask, cv::MORPH_CLOSE, brush);

                double centerOfScreen = WIDTH / 2.0;

                // 5. Find the biggest blue cone on the left side
                std::vector<std::vector<cv::Point>> blueShapes;
                cv::findContours(blueMask, blueShapes, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                double bestBlueX = -1.0;
                double biggestBlueSize = 0.0;

                for (const auto& shape : blueShapes) {
                    double size = cv::contourArea(shape);
                    // Ignore very small noise and very large walls
                    if (size > 40 && size < 5000) {
                        cv::Rect box = cv::boundingRect(shape);
                        double boxCenter = box.x + box.width / 2.0;
                        
                        if (boxCenter < centerOfScreen) {
                            if (size > biggestBlueSize) {
                                biggestBlueSize = size;
                                bestBlueX = boxCenter;
                            }
                        }
                    }
                }

                // 6. Find the biggest yellow cone on the right side
                std::vector<std::vector<cv::Point>> yellowShapes;
                cv::findContours(yellowMask, yellowShapes, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                double bestYellowX = -1.0;
                double biggestYellowSize = 0.0;

                for (const auto& shape : yellowShapes) {
                    double size = cv::contourArea(shape);
                    if (size > 100 && size < 5000) {
                        cv::Rect box = cv::boundingRect(shape);
                        double boxCenter = box.x + box.width / 2.0;

                        if (boxCenter > centerOfScreen) {
                            if (size > biggestYellowSize) {
                                biggestYellowSize = size;
                                bestYellowX = boxCenter;
                            }
                        }
                    }
                }

                // 7. Find where the car should go and remember the road width
                double targetX = centerOfScreen;
                static double savedRoadWidth = 150.0;

                if (bestBlueX != -1.0 && bestYellowX != -1.0) {
                    savedRoadWidth = (bestYellowX - bestBlueX) / 2.0;
                    if (savedRoadWidth < 90.0)  savedRoadWidth = 90.0;
                    if (savedRoadWidth > 220.0) savedRoadWidth = 220.0;
                    targetX = (bestBlueX + bestYellowX) / 2.0;
                }
                else if (bestBlueX != -1.0 && bestYellowX == -1.0) {
                    targetX = bestBlueX + savedRoadWidth;
                }
                else if (bestBlueX == -1.0 && bestYellowX != -1.0) {
                    targetX = bestYellowX - savedRoadWidth;
                }
                else {
                    targetX = centerOfScreen;
                }

                // 8. Calculate turning angle and make it smooth
                double errorPixels = centerOfScreen - targetX;

                double const turnPower = 0.0011; 
                double newAngle = errorPixels * turnPower;

                static double lastSmoothedSteering = 0.0;
                double const smoothLevel = 0.18; 
                double finalAngle = smoothLevel * newAngle + (1.0 - smoothLevel) * lastSmoothedSteering;
                lastSmoothedSteering = finalAngle;

                // Make sure the wheel does not turn too much
                if (finalAngle > 1.0)  finalAngle = 1.0;
                if (finalAngle < -1.0) finalAngle = -1.0;

                // Print the result
                std::cout << "group_21;" << timeMicro << ";" << finalAngle << std::endl;
