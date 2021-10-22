#pragma once
// eigen is a header only library for linear algebra.
// it implements many of the functions commonly used on vectors and matrices
// through the template mechanism it allows for parametric scalar type (int, double, float, whatever you like)
// It supports both fixed size (effiicent) objects and variable size objects.
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>


// we define a template class Isometry_<Scalar, dim> capable of
// storing an SE(dim) object
// Eigen has its own type, for that but here  we make our own as an exercise

// Scalar is the base type used, float, double or something else;
// Dim is the dimension (2 or 3d);
// To materialize a type you will write
// Isometry_<float, 3>
// or
// Isometry_<double, 2>

template <typename Scalar_, int Dim_>
struct Isometry_{
  // when declaring a c++ object containing Eigen types,
  // use this macro that tells the compiler important stuff about the alignment
  // if you compile with optimizations, you will get unexplicable errors otherwise
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // we define a static const member of the class (a global constant) for this type
  // this is an hardcored constant accessible as a static variable in java
  // Isometry<float, 2>::Dim  equals to 2
  // Isometry<double, 3>::Dim  equals to 3
  // this allows us to access the template parameter of the argument from some other object
  
  static constexpr int Dim=Dim_;
  
  // we define set of nested typedefs, that we can access inside the class members
  // same as before
  using Scalar=Scalar_;
  using RotationMatrixType   = Eigen::Matrix<Scalar, Dim, Dim>;
  using TranslatonVectorType = Eigen::Matrix<Scalar, Dim, 1>;
  using MatrixType           = Eigen::Matrix<Scalar, Dim+1, Dim+1>;
  
  // also the class itself is typedeffed, otherwise we would have to write the right hand
  // side of the expression everywhere
  using ThisType = Isometry_<Scalar_, Dim_>;

protected:
  RotationMatrixType   _R;
  TranslatonVectorType _t;

public:
  // default ctor
  Isometry_() {
    _R.setIdentity();
    _t.setZero();
  }

  // ctor from Rotation matrix and Translation vector
  // here we use the initializer list
  Isometry_(const RotationMatrixType& R_, const TranslatonVectorType& t_):
    _R(R_),
    _t(t_){}

  // method that exposes the dim+1 matrix
  // could also give write access through a reference,
  // but the implementation would need to roll back to the defailt matrix object
  MatrixType matrix() const {
    MatrixType m;
    m.setIdentity();
    m.block<Dim,Dim>(0,0)=_R;
    m.block<Dim,1>(0,Dim)=_t;
    return m;
  }

  // trivial accessors for translation and rotation (can write on them)
  TranslatonVectorType& translation() { return _t;}
  RotationMatrixType&   rotation() { return _R;}

  // this is invoked when we do iso1 * iso2 (c++ allows operator overriding)
  ThisType operator *(const ThisType& other_) const {
    return ThisType(_R*other_._R, _t + _R * other_._t);
  }

  // this is invoked when we do iso1 *= iso2 (c++ allows operator overriding)
  ThisType& operator *=(const ThisType& other_) {
    _t += _R * other_._t;
    _R  = _R * other_._R;
  }

  // this is invoked when we do iso1 *= point, and returns a point
  TranslatonVectorType operator*(const TranslatonVectorType& other_){
    return _R * other_ + _t;
  }
  
  // this returns the inverse of an isometry
  ThisType inverse() const {
    return ThisType(_R.transpose(), -_R.transpose()*_t);
  }

};

// some handy typedefs
using Isometry2f=Isometry_<float, 2>;
using Isometry3f=Isometry_<float, 3>;
