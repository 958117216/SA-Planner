#pragma once
#ifndef SOLVER_GUROBI_UTILS_HPP
#define SOLVER_GUROBI_UTILS_HPP

#include "gurobi_c++.h"
#include <sstream>
#include <Eigen/Dense>
#include <type_traits>
// using namespace std;

// custom typedefs
typedef std::vector<GRBLinExpr> GRBVector;
typedef std::vector<std::vector<GRBLinExpr>> GRBMatrix; //外层为行，内层为列

/**
 * @brief 获得vector数集中最小正数
 *
 * @param v 数集
 * @return double
 */
inline double minPositiveElement(std::vector<double> v)
{
    std::sort(v.begin(), v.end());  // sorted in ascending order
    double min_value = 0;
    for (int i = 0; i < v.size(); i++)
    {
        if (v[i] > 0)
        {
            min_value = v[i];
            break;
        }
    }
    return min_value;
}

/**
 * @brief Get the Norm2 object
 *
 * @tparam T
 * @param x
 * @return GRBQuadExpr
 */
template <typename T>
GRBQuadExpr getNorm2(const std::vector<T>& x)  // Return the squared norm of a vector
{
    GRBQuadExpr result = 0;
    for (int i = 0; i < x.size(); i++)
    {
        result = result + x[i] * x[i];
    }
    return result;
}

/**
 * @brief 为GRBModel 添加等式约束
 *
 * @param m
 * @param a
 * @param b
 */
inline void addVectorEqConstraint(GRBModel& m, const GRBVector a, const Eigen::Vector3d& b)
{
    for (int i = 0; i < a.size(); i++)
    {
        m.addConstr(a[i] == b[i]);
    }
}

/**
 * @brief 为GRBModel 添加 小于等于 约束
 *
 * @param m
 * @param a
 * @param b
 */
inline void addVectorLessEqualConstraint(GRBModel& m, const GRBVector a, const Eigen::Vector3d& b)
{
    for (int i = 0; i < a.size(); i++)
    {
        m.addConstr(a[i] <= b[i]);
    }
}

/**
 * @brief 为GRBModel 添加 大于等于 约束
 *
 * @param m
 * @param a
 * @param b
 */
inline void addVectorGreaterEqualConstraint(GRBModel& m, const GRBVector a, const Eigen::Vector3d& b)
{
    for (int i = 0; i < a.size(); i++)
    {
        m.addConstr(a[i] >= b[i]);
    }
}

/**
 * @brief  完全重置GRBModel
 *
 * @param m
 */
inline void resetCompleteModel(GRBModel& m)
{
    GRBConstr* c = 0;
    c = m.getConstrs();
    for (int i = 0; i < m.get(GRB_IntAttr_NumConstrs); ++i)
    {
        m.remove(c[i]);
    }

    GRBQConstr* cq = 0;
    cq = m.getQConstrs();
    for (int i = 0; i < m.get(GRB_IntAttr_NumQConstrs); ++i)
    {
        m.remove(cq[i]);
    }

    GRBGenConstr* gc = 0;
    gc = m.getGenConstrs();
    for (int i = 0; i < m.get(GRB_IntAttr_NumGenConstrs); ++i)
    {
        m.remove(gc[i]);
    }

    GRBVar* vars = 0;
    vars = m.getVars();
    for (int i = 0; i < m.get(GRB_IntAttr_NumVars); ++i)
    {
        m.remove(vars[i]);
    }

    m.reset();  // Note that this function, only by itself, does NOT remove vars or constraints
    //Reset the model to an unsolved state, discarding any previously computed solution information.
   m.update(); //模型的变量和约束的添加和删除不会立即生效
}

// See https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html#sec:StatusCodes
/**
 * @brief 打印Gurobi求解的状态
 *
 * @param status
 */
inline void printGurobiStatus(int status)
{
    switch (status)
    {
        case GRB_LOADED:
            std::cout << "GUROBI Status: GRB_LOADED" << std::endl;
            break;
        case GRB_OPTIMAL:
            std::cout << "GUROBI Status: GRB_OPTIMAL" << std::endl;
            break;
        case GRB_INFEASIBLE:
            std::cout << "GUROBI Status: GRB_INFEASIBLE" << std::endl;
            break;
        case GRB_INF_OR_UNBD:
            std::cout << "GUROBI Status: GRB_INF_OR_UNBD" << std::endl;
            break;
        case GRB_UNBOUNDED:
            std::cout << "GUROBI Status: GRB_UNBOUNDED" << std::endl;
            break;
        case GRB_CUTOFF:
            std::cout << "GUROBI Status: GRB_CUTOFF" << std::endl;
            break;
        case GRB_ITERATION_LIMIT:
            std::cout << "GUROBI Status: GRB_ITERATION_LIMIT" << std::endl;
            break;
        case GRB_NODE_LIMIT:
            std::cout << "GUROBI Status: GRB_NODE_LIMIT" << std::endl;
            break;
        case GRB_TIME_LIMIT:
            std::cout << "GUROBI Status: GRB_TIME_LIMIT" << std::endl;
            break;
        case GRB_SOLUTION_LIMIT:
            std::cout << "GUROBI Status: GRB_SOLUTION_LIMIT" << std::endl;
            break;
        case GRB_INTERRUPTED:
            std::cout << "GUROBI Status: GRB_INTERRUPTED" << std::endl;
            break;
        case GRB_NUMERIC:
            std::cout << "GUROBI Status: GRB_NUMERIC" << std::endl;
            break;
        case GRB_SUBOPTIMAL:
            std::cout << "GUROBI Status: GRB_SUBOPTIMAL" << std::endl;
            break;
        case GRB_INPROGRESS:
            std::cout << "GUROBI Status: GRB_INPROGRESS" << std::endl;
            break;
        case GRB_USER_OBJ_LIMIT:
            std::cout << "GUROBI Status: GRB_USER_OBJ_LIMIT" << std::endl;
            break;
        default:
            std::cout << "GUROBI Status Code=: " << status << std::endl;
    }
}

template <typename T, typename R>
GRBVector matrixMultiply(const std::vector<std::vector<R>>& A, const std::vector<T>& x)
{
    GRBVector result;

    for (int i = 0; i < A.size(); i++)
    {
        GRBLinExpr lin_exp = 0;
        for (int m = 0; m < x.size(); m++)
        {
            lin_exp = lin_exp + A[i][m] * x[m];
        }
        result.push_back(lin_exp);
    }
    return result;
}

template <typename T>
std::vector<GRBVector> matrixMultiply(const std::vector<std::vector<T>>& A, const std::vector<std::vector<double>>& B)
{
    std::vector<GRBVector> result(A.size(), GRBVector(B[0].size(), 0.0));  // Initialize all the
    // elements to zero

    for (int i = 0; i < A.size(); i++)  // multiply row if of A
    {
        for (int j = 0; j < B[0].size(); j++)  // times column j of B
        {
            GRBLinExpr lin_exp = 0;
            for (int m = 0; m < B.size(); m++)
            {
                lin_exp += A[i][m] * B[m][j];
            }
            result[i][j] = lin_exp;
        }
    }
    return result;
}

//重载std::vector 的+(按位加法) -(按位减法) *(数乘)
template <typename T>  // Overload + to sum Elementwise std::vectors
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b)
{
    assert(a.size() == b.size());

    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), b.begin(), std::back_inserter(result), std::plus<T>());
    return result;
}

template <typename T>  // Overload - to substract Elementwise std::vectors
std::vector<T> operator-(const std::vector<T>& a, const std::vector<T>& b)
{
    assert(a.size() == b.size());

    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), b.begin(), std::back_inserter(result), std::minus<T>());
    return result;
}

template <typename T>  // Overload *
std::vector<T> operator*(const double& a, const std::vector<T>& b)
{
    std::vector<T> result;

    for (int i = 0; i < b.size(); i++)
    {
        result.push_back(a * b[i]);
    }

    return result;
}

template <typename T>
GRBVector operator-(const std::vector<T>& x, const std::vector<double>& b)
{
    GRBVector result;
    for (int i = 0; i < x.size(); i++)
    {
        GRBLinExpr tmp = x[i] - b[i];
        result.push_back(tmp);
    }
    return result;
}

template <typename T>
std::vector<T> eigenVector2std(const Eigen::Matrix<T, -1, 1>& x)
{
    std::vector<T> result;
    for (int i = 0; i < x.rows(); i++)
    {
        result.push_back(x(i, 0));
    }
    return result;
}

template <typename T>
std::vector<T> eigenVector2std(const Eigen::Matrix<T, 3, 1>& x)  // TODO: Merge with the previous one?
{
    std::vector<T> result;
    for (int i = 0; i < x.rows(); i++)
    {
        result.push_back(x(i,0));
    }
    return result;
}

inline std::vector<std::vector<double>> eigenMatrix2std(const Eigen::Matrix<double, -1, -1>& x)
{
    std::vector<std::vector<double>> result;

    for (int i = 0; i < x.rows(); i++)
    {
        std::vector<double> row;
        for (int j = 0; j < x.cols(); j++)
        {
            row.push_back(x(i, j));
        }
        result.push_back(row);
    }
    return result;
}

template <typename T>
std::vector<T> getColumn(std::vector<std::vector<T>> x, int column)
{
    std::vector<T> result;

    for (int i = 0; i < x.size(); i++)
    {
        result.push_back(x[i][column]);
    }
    return result;
}


#endif
