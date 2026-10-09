#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"

const std::string keys =
    "{help h usage ? |                          | 输出命令行参数说明}"
    "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
    "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

std::vector<cv::Point3f> centers_3d(const cv::Size &pattern_size,
                                    const float center_distance) {
  std::vector<cv::Point3f> centers_3d;

  for (int i = 0; i < pattern_size.height; i++)
    for (int j = 0; j < pattern_size.width; j++)
      centers_3d.push_back({j * center_distance, i * center_distance, 0});

  return centers_3d;
}

void load(const std::string &input_folder, const std::string &config_path,
          cv::Size &img_size, std::vector<std::vector<cv::Point3f>> &obj_points,
          std::vector<std::vector<cv::Point2f>> &img_points) {
  // 读取yaml参数
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();
  cv::Size pattern_size(pattern_cols, pattern_rows);

  for (int i = 1; true; i++) {
    // 读取图片
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty())
      break;

    // 设置图片尺寸
    img_size = img.size();

    // 识别标定板
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findChessboardCorners(gray, pattern_size, centers_2d);
    if (success)
      cv::cornerSubPix(
          gray, centers_2d, {11, 11}, {-1, -1},
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                           0.001));

    // 显示识别结果
    auto drawing = img.clone();
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5); // 缩小图片尺寸便于显示完全
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(0);

    // 输出识别结果
    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (!success)
      continue;

    // 记录所需的数据
    img_points.emplace_back(centers_2d);
    obj_points.emplace_back(centers_3d(pattern_size, center_distance_mm));
  }
}

// ChArUco 标定板加载函数
void load_charuco(const std::string &input_folder,
                  const std::string &config_path, cv::Size &img_size,
                  std::vector<std::vector<cv::Point3f>> &obj_points,
                  std::vector<std::vector<cv::Point2f>> &img_points) {
  // 读取yaml参数
  auto yaml = YAML::LoadFile(config_path);
  int squares_x = yaml["charuco_squares_x"].as<int>();
  int squares_y = yaml["charuco_squares_y"].as<int>();
  float square_length_mm = yaml["charuco_square_length_mm"].as<float>();
  float marker_length_mm = yaml["charuco_marker_length_mm"].as<float>();
  int dict_id = yaml.value("charuco_dict", 8); // 默认 DICT_6X6_250

  // 转换为米
  float square_length_m = square_length_mm / 1000.0f;
  float marker_length_m = marker_length_mm / 1000.0f;

  // 创建 ArUco 字典和 ChArUco 板
  auto dictionary = cv::aruco::getPredefinedDictionary(dict_id);
  auto board = cv::aruco::CharucoBoard::create(
      squares_x, squares_y, square_length_m, marker_length_m, dictionary);

  // ChArUco 内部角点数
  int total_corners = (squares_x - 1) * (squares_y - 1);

  fmt::print("[ChArUco] 板参数: {}x{}, 方格={}mm, marker={}mm, dict={}\n",
             squares_x, squares_y, square_length_mm, marker_length_mm, dict_id);
  fmt::print("[ChArUco] 期望角点数: {}\n", total_corners);

  for (int i = 1; true; i++) {
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty())
      break;

    img_size = img.size();

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // 1. 检测 ArUco markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dictionary, corners, ids);

    auto drawing = img.clone();
    bool success = false;

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(drawing, corners, ids);

      // 2. 插值 ChArUco 角点
      std::vector<cv::Point2f> charuco_corners;
      std::vector<int> charuco_ids;
      cv::aruco::interpolateCornersCharuco(corners, ids, gray, board,
                                           charuco_corners, charuco_ids);

      int detected = charuco_corners.size();
      if (detected >= total_corners * 0.3) { // 至少 30% 的角点
        cv::aruco::drawDetectedCornersCharuco(drawing, charuco_corners);

        // 3. 构建 3D-2D 对应点
        std::vector<cv::Point3f> matched_3d;
        std::vector<cv::Point2f> matched_2d;

        for (size_t j = 0; j < charuco_ids.size(); j++) {
          int corner_id = charuco_ids[j];
          if (corner_id >= 0 && corner_id < (int)board->objPoints.size()) {
            matched_3d.push_back(board->objPoints[corner_id]);
            matched_2d.push_back(charuco_corners[j]);
          }
        }

        if (matched_3d.size() >= 4) {
          obj_points.emplace_back(matched_3d);
          img_points.emplace_back(matched_2d);
          success = true;
          fmt::print("[{}] {} ({}角点, {}/{})\n", i, "OK", matched_3d.size(),
                     detected, total_corners);
        } else {
          fmt::print("[{}] 角点不足: {}\n", i, matched_3d.size());
        }
      } else {
        fmt::print("[{}] 检测到角点太少: {}/{}\n", i, detected, total_corners);
      }
    } else {
      fmt::print("[{}] 未检测到 marker\n", i);
    }

    cv::resize(drawing, drawing, {}, 0.5, 0.5);
    cv::imshow("ChArUco Calibration - Press any key", drawing);
    cv::waitKey(0);
  }
}

void print_yaml(const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs,
                double error) {
  YAML::Emitter result;
  std::vector<double> camera_matrix_data(camera_matrix.begin<double>(),
                                         camera_matrix.end<double>());
  std::vector<double> distort_coeffs_data(distort_coeffs.begin<double>(),
                                          distort_coeffs.end<double>());

  result << YAML::BeginMap;
  result << YAML::Comment(fmt::format("重投影误差: {:.4f}px", error));
  result << YAML::Key << "camera_matrix";
  result << YAML::Value << YAML::Flow << camera_matrix_data;
  result << YAML::Key << "distort_coeffs";
  result << YAML::Value << YAML::Flow << distort_coeffs_data;
  result << YAML::Newline;
  result << YAML::EndMap;

  fmt::print("\n{}\n", result.c_str());
}

int main(int argc, char *argv[]) {
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");

  // 从输入文件夹中加载标定所需的数据
  cv::Size img_size;
  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;

  // 读取配置，判断使用 ChArUco 还是棋盘格
  auto yaml = YAML::LoadFile(config_path);
  bool use_charuco = yaml.value("use_charuco", false);

  if (use_charuco) {
    fmt::print("使用 ChArUco 标定板\n");
    load_charuco(input_folder, config_path, img_size, obj_points, img_points);
  } else {
    fmt::print("使用棋盘格标定板\n");
    load(input_folder, config_path, img_size, obj_points, img_points);
  }

  if (obj_points.empty()) {
    fmt::print("错误: 没有有效的标定数据！\n");
    return -1;
  }

  // 相机标定
  cv::Mat camera_matrix, distort_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  auto criteria = cv::TermCriteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
      DBL_EPSILON); // 默认迭代次数(30)有时会导致结果发散，故设为100
  cv::calibrateCamera(obj_points, img_points, img_size, camera_matrix,
                      distort_coeffs, rvecs, tvecs, cv::CALIB_FIX_K3,
                      criteria); // 由于视场角较小，不需要考虑k3

  // 重投影误差
  double error_sum = 0;
  size_t total_points = 0;
  for (size_t i = 0; i < obj_points.size(); i++) {
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix,
                      distort_coeffs, reprojected_points);

    total_points += reprojected_points.size();
    for (size_t j = 0; j < reprojected_points.size(); j++)
      error_sum += cv::norm(img_points[i][j] - reprojected_points[j]);
  }
  auto error = error_sum / total_points;

  // 输出yaml
  print_yaml(camera_matrix, distort_coeffs, error);
}
