#include <gtest/gtest.h>

#include <pal_cereal/pal_cereal.h>
#include <pal_cereal/Eigen/cerealization_eigen.h>

class TestCerealizationEigenCore : public testing::Test
{
public:

  TestCerealizationEigenCore()
  {
    initData();
  }

  std::string file = "/tmp/random_eigen";
  std::vector<std::string> extensions = {".bin", ".xml", ".json"};

  Eigen::Matrix<int, 10, 1> vector_int_fixed;
  Eigen::Matrix<int, 10, 10> matrix_int_fixed;

  Eigen::Matrix<float, 10, 1> vector_float_fixed;
  Eigen::Matrix<float, 10, 10> matrix_float_fixed;

  Eigen::Matrix<double, 10, 1> vector_double_fixed;
  Eigen::Matrix<double, 10, 10> matrix_double_fixed;

  Eigen::Matrix<int, Eigen::Dynamic, 1> vector_int_dyn;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> matrix_int_dyn;

  Eigen::Matrix<float, Eigen::Dynamic, 1> vector_float_dyn;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix_float_dyn;

  Eigen::Matrix<double, Eigen::Dynamic, 1> vector_double_dyn;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_double_dyn;

private:

  void initData()
  {
    vector_int_fixed.setRandom();
    matrix_int_fixed.setRandom();

    vector_float_fixed.setRandom();
    matrix_float_fixed.setRandom();

    vector_double_fixed.setRandom();
    matrix_double_fixed.setRandom();

    vector_int_dyn = Eigen::Matrix<int, Eigen::Dynamic, 1>::Random(10);
    matrix_int_dyn = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>::Random(10, 10);

    vector_float_dyn = Eigen::Matrix<float, Eigen::Dynamic, 1>::Random(10);
    matrix_float_dyn = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Random(10, 10);

    vector_double_dyn = Eigen::Matrix<double, Eigen::Dynamic, 1>::Random(10);
    matrix_double_dyn = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Random(10, 10);
  }
};

TEST_F(TestCerealizationEigenCore, CerealizationEigenCoreSaveLoad)
{
  Eigen::Matrix<int, 10, 1> vector_int_fixed_load;
  Eigen::Matrix<int, 10, 10> matrix_int_fixed_load;

  Eigen::Matrix<float, 10, 1> vector_float_fixed_load;
  Eigen::Matrix<float, 10, 10> matrix_float_fixed_load;

  Eigen::Matrix<double, 10, 1> vector_double_fixed_load;
  Eigen::Matrix<double, 10, 10> matrix_double_fixed_load;

  Eigen::Matrix<int, Eigen::Dynamic, 1> vector_int_dyn_load;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> matrix_int_dyn_load;

  Eigen::Matrix<float, Eigen::Dynamic, 1> vector_float_dyn_load;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix_float_dyn_load;

  Eigen::Matrix<double, Eigen::Dynamic, 1> vector_double_dyn_load;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_double_dyn_load;

  for (const auto& ext : extensions)
  {
    const std::string file_name = file + ext;

    std::cout << "Testing extension " << ext << "\n";

    /// Save/Load fixed type

    EXPECT_NO_THROW(pal::save(file_name, vector_int_fixed));
    EXPECT_NO_THROW(pal::load(file_name, vector_int_fixed_load));
    EXPECT_EQ(vector_int_fixed, vector_int_fixed_load);

    EXPECT_NO_THROW(pal::save(file_name, matrix_int_fixed));
    EXPECT_NO_THROW(pal::load(file_name, matrix_int_fixed_load));
    EXPECT_EQ(matrix_int_fixed, matrix_int_fixed_load);

    EXPECT_NO_THROW(pal::save(file_name, vector_float_fixed));
    EXPECT_NO_THROW(pal::load(file_name, vector_float_fixed_load));
    EXPECT_EQ(vector_float_fixed, vector_float_fixed_load);

    EXPECT_NO_THROW(pal::save(file_name, matrix_float_fixed));
    EXPECT_NO_THROW(pal::load(file_name, matrix_float_fixed_load));
    EXPECT_EQ(matrix_float_fixed, matrix_float_fixed_load);

    EXPECT_NO_THROW(pal::save(file_name, vector_double_fixed));
    EXPECT_NO_THROW(pal::load(file_name, vector_double_fixed_load));
    EXPECT_EQ(vector_double_fixed, vector_double_fixed_load);

    EXPECT_NO_THROW(pal::save(file_name, matrix_double_fixed));
    EXPECT_NO_THROW(pal::load(file_name, matrix_double_fixed_load));
    EXPECT_EQ(matrix_double_fixed, matrix_double_fixed_load);

    /// Save/Load dynamic type

    EXPECT_NO_THROW(pal::save(file_name, vector_int_dyn));
    EXPECT_NO_THROW(pal::load(file_name, vector_int_dyn_load));
    EXPECT_EQ(vector_int_dyn, vector_int_dyn_load);

    EXPECT_NO_THROW(pal::save(file_name, matrix_int_dyn));
    EXPECT_NO_THROW(pal::load(file_name, matrix_int_dyn_load));
    EXPECT_EQ(matrix_int_dyn, matrix_int_dyn_load);

    EXPECT_NO_THROW(pal::save(file_name, vector_float_dyn));
    EXPECT_NO_THROW(pal::load(file_name, vector_float_dyn_load));
    EXPECT_EQ(vector_float_dyn, vector_float_dyn_load);

    EXPECT_NO_THROW(pal::save(file_name, matrix_float_dyn));
    EXPECT_NO_THROW(pal::load(file_name, matrix_float_dyn_load));
    EXPECT_EQ(matrix_float_dyn, matrix_float_dyn_load);

    EXPECT_NO_THROW(pal::save(file_name, vector_double_dyn));
    EXPECT_NO_THROW(pal::load(file_name, vector_double_dyn_load));
    EXPECT_EQ(vector_double_dyn, vector_double_dyn_load);

    EXPECT_NO_THROW(pal::save(file_name, matrix_double_dyn));
    EXPECT_NO_THROW(pal::load(file_name, matrix_double_dyn_load));
    EXPECT_EQ(matrix_double_dyn, matrix_double_dyn_load);

    /// Save fixed / load dynamic type

    EXPECT_NO_THROW(pal::save(file_name, vector_int_fixed));
    EXPECT_NO_THROW(pal::load(file_name, vector_int_dyn_load));
    EXPECT_EQ(vector_int_fixed, vector_int_dyn_load);

    EXPECT_NO_THROW(pal::save(file_name, matrix_int_fixed));
    EXPECT_NO_THROW(pal::load(file_name, matrix_int_fixed_load));
    EXPECT_EQ(matrix_int_fixed, matrix_int_fixed_load);

    EXPECT_NO_THROW(pal::save(file_name, vector_float_fixed));
    EXPECT_NO_THROW(pal::load(file_name, vector_float_dyn_load));
    EXPECT_EQ(vector_float_fixed, vector_float_dyn_load);

    EXPECT_NO_THROW(pal::save(file_name, matrix_float_fixed));
    EXPECT_NO_THROW(pal::load(file_name, matrix_float_dyn_load));
    EXPECT_EQ(matrix_float_fixed, matrix_float_dyn_load);

    EXPECT_NO_THROW(pal::save(file_name, vector_double_fixed));
    EXPECT_NO_THROW(pal::load(file_name, vector_double_dyn_load));
    EXPECT_EQ(vector_double_fixed, vector_double_dyn_load);

    EXPECT_NO_THROW(pal::save(file_name, matrix_double_fixed));
    EXPECT_NO_THROW(pal::load(file_name, matrix_double_dyn_load));
    EXPECT_EQ(matrix_double_fixed, matrix_double_dyn_load);

    /// Save dynamic / load fixed type

    EXPECT_NO_THROW(pal::save(file_name, vector_int_dyn));
    EXPECT_NO_THROW(pal::load(file_name, vector_int_fixed_load));
    EXPECT_EQ(vector_int_dyn, vector_int_fixed_load);

    EXPECT_NO_THROW(pal::save(file_name, matrix_int_dyn));
    EXPECT_NO_THROW(pal::load(file_name, matrix_int_fixed_load));
    EXPECT_EQ(matrix_int_dyn, matrix_int_fixed_load);

    EXPECT_NO_THROW(pal::save(file_name, vector_float_dyn));
    EXPECT_NO_THROW(pal::load(file_name, vector_float_fixed_load));
    EXPECT_EQ(vector_float_dyn, vector_float_fixed_load);

    EXPECT_NO_THROW(pal::save(file_name, matrix_float_dyn));
    EXPECT_NO_THROW(pal::load(file_name, matrix_float_fixed_load));
    EXPECT_EQ(matrix_float_dyn, matrix_float_fixed_load);

    EXPECT_NO_THROW(pal::save(file_name, vector_double_dyn));
    EXPECT_NO_THROW(pal::load(file_name, vector_double_fixed_load));
    EXPECT_EQ(vector_double_dyn, vector_double_fixed_load);

    EXPECT_NO_THROW(pal::save(file_name, matrix_double_dyn));
    EXPECT_NO_THROW(pal::load(file_name, matrix_double_fixed_load));
    EXPECT_EQ(matrix_double_dyn, matrix_double_fixed_load);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
