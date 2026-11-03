#include <fstream>
#include <iomanip>
#include <iostream>

#include "crobot/optimization/problem.hpp"

using namespace crobot;
using namespace optimization;

//==============================================================================
Problem::Problem()
  : m_constraints("constraint-sets", false), m_costs("cost-terms", true) {
  m_variables = std::make_shared<Composite>("variable-sets", false);
}

//==============================================================================
void Problem::add_variable_set(VariableSet::Ptr variable_set) {
  m_variables->add_component(variable_set);
}

//==============================================================================
void Problem::add_constraint_set(ConstraintSet::Ptr constraint_set) {
  constraint_set->link_with_variables(m_variables);
  m_constraints.add_component(constraint_set);
}

//==============================================================================
void Problem::add_cost_set(CostTerm::Ptr cost_set) {
  cost_set->link_with_variables(m_variables);
  m_costs.add_component(cost_set);
}

//==============================================================================
int Problem::get_num_variables() const {
  return m_variables->get_rows();
}

//==============================================================================
VecBound Problem::get_variable_bounds() const {
  return m_variables->get_bounds();
}

//==============================================================================
math::VectorXd Problem::get_variable_values() const {
  return m_variables->get_values();
}

//==============================================================================
void Problem::set_variables(const double* x) {
  m_variables->set_variables(convert_to_Eigen(x));
}

//==============================================================================
double Problem::evaluate_cost_function(const double* x) {
  math::VectorXd g = math::VectorXd::Zero(1);
  if (has_cost_terms()) {
    set_variables(x);
    g = m_costs.get_values();
  }
  return g(0);
}

//==============================================================================
math::VectorXd Problem::evaluate_cost_function_gradient(
    const double* x, bool use_finite_difference_approximation, double epsilon) {
  int n                    = get_num_variables();
  math::SparseJacobian jac = math::SparseJacobian(1, n);
  if (has_cost_terms()) {
    if (use_finite_difference_approximation) {
      double step_size = epsilon;

      // calculate forward difference by disturbing each optimization variable
      double g = evaluate_cost_function(x);
      std::vector<double> x_new(x, x + n);
      for (int i = 0; i < n; ++i) {
        x_new[i] += step_size; // disturb
        double g_new       = evaluate_cost_function(x_new.data());
        jac.coeffRef(0, i) = (g_new - g) / step_size;
        x_new[i]           = x[i]; // reset for next iteration
      }
    }
    else {
      set_variables(x);
      jac = m_costs.get_jacobian();
    }
  }

  return jac.row(0).transpose();
}

//==============================================================================
VecBound Problem::get_constraint_bounds() const {
  return m_constraints.get_bounds();
}

//==============================================================================
int Problem::get_num_constraints() const {
  return get_constraint_bounds().size();
}

//==============================================================================
math::VectorXd Problem::evaluate_constraints(const double* x) {
  set_variables(x);
  return m_constraints.get_values();
}

//==============================================================================
bool Problem::has_cost_terms() const {
  return m_costs.get_rows() > 0;
}

//==============================================================================
void Problem::evaluate_jacobian_nonzeros(const double* x, double* values) {
  set_variables(x);
  math::SparseJacobian jac = get_constraint_jacobian();

  jac.makeCompressed(); // so the valuePtr() is dense and accurate
  std::copy(jac.valuePtr(), jac.valuePtr() + jac.nonZeros(), values);
}

//==============================================================================
math::SparseJacobian Problem::get_constraint_jacobian() const {
  return m_constraints.get_jacobian();
}

//==============================================================================
math::SparseJacobian Problem::get_cost_jacobian() const {
  return m_costs.get_jacobian();
}

//==============================================================================
void Problem::save_current() {
  x_prev.push_back(m_variables->get_values());
}

//==============================================================================
Composite::Ptr Problem::get_variables() const {
  return m_variables;
}

//==============================================================================
void Problem::set_variables(int iter) {
  m_variables->set_variables(x_prev.at(iter));
}

//==============================================================================
void Problem::set_variable_final() {
  m_variables->set_variables(x_prev.at(get_iter_count() - 1));
}

//==============================================================================
void Problem::print_current() const {
  std::cout
      << "\n"
      << "Legend:\n"
      << "c - number of variables, constraints or cost terms" << std::endl
      << "i - indices of this set in overall problem" << std::endl
      << "v - number of [violated variable- or constraint-bounds] or [cost "
         "term value]"
      << "\n\n"
      << std::right << std::setw(33) << "" << std::setw(5) << "c  "
      << std::setw(16) << "i    " << std::setw(11) << "v " << std::left << "\n";

  m_variables->print_all();
  m_constraints.print_all();
  m_costs.print_all();
};

//==============================================================================
math::VectorXd Problem::convert_to_Eigen(const double* x) const {
  return Eigen::Map<const math::VectorXd>(x, get_num_variables());
}

//==============================================================================
void Problem::save_to_file(const std::string& filename) {
  // https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
  const static Eigen::IOFormat CSVFormat(
      Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

  std::ofstream file(filename);
  if (file.is_open()) {
    file << m_variables->get_values().format(CSVFormat);
    file.close();
  }
}

//==============================================================================
void Problem::load_from_file(const std::string& filename) {

  // the inspiration for creating this function was drawn from here (I did NOT
  // copy and paste the code)
  // https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix

  // the input is the file: "filename.csv":
  // a,b,c
  // d,e,f
  // This function converts input file data into the Eigen matrix format

  // the matrix entries are stored in this variable row-wise. For example if we
  // have the matrix: M=[a b c
  //    d e f]
  // the entries are stored as matrixEntries=[a,b,c,d,e,f], that is the variable
  // "matrixEntries" is a row vector later on, this vector is mapped into the
  // Eigen matrix format
  std::vector<double> matrixEntries;

  // in this object we store the data from the matrix
  std::ifstream matrixDataFile(filename);

  // this variable is used to store the row of the matrix that contains commas
  std::string matrixRowString;

  // this variable is used to store the matrix entry;
  std::string matrixEntry;

  // this variable is used to track the number of rows
  int matrixRowNumber = 0;

  // here we read a row by row of matrixDataFile and store
  // every line into the string variable matrixRowString
  while (getline(matrixDataFile, matrixRowString)) {
    std::stringstream matrixRowStringStream(matrixRowString);
    // convert matrixRowString that is a string to a
    // stream variable.

    while (getline(matrixRowStringStream, matrixEntry, ','))
    // here we read pieces of the stream matrixRowStringStream until
    // every comma, and store the resulting character into the
    // matrixEntry
    {
      matrixEntries.push_back(stod(matrixEntry));
      // here we convert the string to double and fill
      // in the row vector storing all the matrix entries
    }
    matrixRowNumber++; // update the column numbers
  }

  // here we convet the vector variable into the matrix and return the resulting
  // object, note that matrixEntries.data() is the pointer to the first memory
  // location at which the entries of the vector matrixEntries are stored;
  math::MatrixXd variables = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      matrixEntries.data(),
      matrixRowNumber,
      matrixEntries.size() / matrixRowNumber);
  CROBOT_ASSERT_ERROR(variables.cols() == 1);
  CROBOT_ASSERT_ERROR(variables.rows() == get_num_variables());
  m_variables->set_variables(variables);
}
