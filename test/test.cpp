#include <gtest/gtest.h>
#include <ros/package.h>
#include "semantics_manager/semantic_color_lut.h"

TEST(ASOOM_utils, test_sem_color_lut) {
  SemanticColorLut lut(ros::package::getPath("semantics_manager") + "/classes/test_classes.yaml");  
  // Some unknown class
  auto color = SemanticColorLut::unpackColor(lut.ind2Color(34));
  EXPECT_EQ(color[0], 0);
  EXPECT_EQ(color[1], 0);
  EXPECT_EQ(color[2], 0);
  EXPECT_EQ(lut.color2Ind(SemanticColorLut::packColor(color[0], color[1], color[2])), 255);
  color = SemanticColorLut::unpackColor(lut.ind2Color(0));
  EXPECT_EQ(color[0], 255);
  EXPECT_EQ(color[1], 0);
  EXPECT_EQ(color[2], 0);
  EXPECT_EQ(lut.color2Ind(SemanticColorLut::packColor(color[0], color[1], color[2])), 0);

  cv::Mat img = cv::Mat::zeros(1000, 1000, CV_8UC1);
  for (uint8_t i=0; i<=10; i++) {
    img.at<uint8_t>(i, 0) = i;
  }
  cv::Mat color_img;
  lut.ind2Color(img, color_img);
  EXPECT_EQ(color_img.type(), CV_8UC3);
  // These are BGR because OpenCV
  EXPECT_EQ(color_img.at<cv::Vec3b>(0, 0), cv::Vec3b(255, 0, 0));
  EXPECT_EQ(color_img.at<cv::Vec3b>(1, 0), cv::Vec3b(0, 255, 0));
  EXPECT_EQ(color_img.at<cv::Vec3b>(3, 0), cv::Vec3b(0, 100, 0));
  EXPECT_EQ(color_img.at<cv::Vec3b>(4, 0), cv::Vec3b(255, 255, 0));
  EXPECT_EQ(color_img.at<cv::Vec3b>(5, 0), cv::Vec3b(255, 0, 255));
  EXPECT_EQ(color_img.at<cv::Vec3b>(10, 0), cv::Vec3b(0, 0, 0));

  // Go backwards
  lut.color2Ind(color_img, img);
  EXPECT_EQ(img.type(), CV_8UC1);
  for (uint8_t i=0; i<6; i++) {
    EXPECT_EQ(img.at<uint8_t>(i, 0), i);
  }
  EXPECT_EQ(img.at<uint8_t>(10, 0), 255);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
