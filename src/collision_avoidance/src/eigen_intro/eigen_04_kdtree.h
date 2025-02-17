#pragma once
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
#include "eigen_03_covariance.h"
#include <memory>
#include <iterator>

using namespace std;

// split functions
// takes two iterators (begin and end)
// and reorders the data so that the items for which the predicate
// is true are at the beginning, tho others at the end
// it returns the iterator to the splitting element between the two classes
template <typename IteratorType_, typename PredicateType_>
IteratorType_ split(IteratorType_ begin,
                    IteratorType_ end,
                    PredicateType_ predicate) {
  using ValueType    = typename IteratorType_::value_type;
  auto lower=begin;  //first
  // upper is an iterator at the end, it is a reverse iterator,
  // and moves backward when incremented;
  // we apply the predicate to each point,
  // and if the result is positive
  //   we leave the point where it is
  // otherwise
  //   we move the point in the other extrema, and increment the extrema
  // the iterations end when the upper and lower ends match
  auto upper=std::make_reverse_iterator(end); 
  while (lower!=upper.base()) {
    ValueType& v_lower=*lower;
    ValueType& v_upper=*upper;
    if ( predicate(v_lower) ){
      ++lower;
    } else {
      std::swap(v_lower,v_upper);
      ++upper;
    }
  }
  return upper.base();
}

// base class of a kd tree node
// has a static "build" function, and operates
// see the *_test.cpp file on how to use this class

template <typename ContainerType_>
struct TreeNodeBase_ {
  using ContainerType=ContainerType_;
  using IteratorType=typename ContainerType_::iterator;
  
  // value type is a typedef member of a container, that tells the type of the contained type
  using VectorType= typename ContainerType_::value_type;
  // Eigen vectors offer a static const value that tells the dimensions
  static constexpr int Dim = VectorType::RowsAtCompileTime;
  using Scalar = typename VectorType::Scalar;
  using BaseType = TreeNodeBase_<ContainerType_>;
  using MatrixType = Eigen::Matrix<Scalar, Dim, Dim>;


  TreeNodeBase_(IteratorType begin_,
                IteratorType end_,
                int num_points_):
    _begin(begin_),
    _end(end_),
    _num_points(num_points_){
  }

  // pure virtual function for search, specialized in descendants
  virtual void search(ContainerType& answers, const VectorType& query, float squared_norm) const = 0;

  // static function that builds the tree.
  // the c++ library allows for managed pointers
  // shared: the contained object can be possessed by multiple owners
  //         when the last owner is destroyed, the owned object is destroyed too
  //         this mechanism is breaks when cycles are introduced in the ownership
  // weak:   these pointers are "weak copy of shared". A weak pointer is linked to a shared
  //         but does not own the object
  // unique: no copy allowed, only one owner.
  
  static std::unique_ptr<BaseType> buildTree(IteratorType begin, IteratorType end, int max_points_in_leaf);
  using TreeNodeBasePtr=std::unique_ptr<BaseType>;

  IteratorType _begin;
  IteratorType _end;
  int _num_points;

};

template<typename ContainerType_>
using TreeNodeBasePtr_ = std::unique_ptr<TreeNodeBase_<ContainerType_> >;


template  <typename TreeNodeBaseType_>
struct TreeNodeLeaf_: public TreeNodeBaseType_ {
  using BaseType   = TreeNodeBaseType_;
  using Scalar     = typename BaseType::Scalar;
  using VectorType = typename BaseType::VectorType;
  using MatrixType = typename BaseType::BaseType;
  using ContainerType   = typename BaseType::ContainerType;
  using IteratorType=typename BaseType::IteratorType;
  static constexpr int Dim = BaseType::Dim_;

  // does a brute force search selecting all items at smallest distance
  void search(ContainerType& answers, const VectorType& query, float squared_norm) const override {
    for (auto it=this->_begin; it!=this->_end; ++it){
      const VectorType& v=*it;
      if ((query-v).squaredNorm()<squared_norm)
        answers.push_back(v);
    }
  }
  TreeNodeLeaf_(IteratorType begin_,
                IteratorType end_,
                int num_points_): BaseType(begin_, end_, num_points_){
    std::cerr << "leaf (" << this->_num_points << ")";
  }
};


template <typename TreeNodeBaseType_>
struct TreeNodeMiddle_:  public TreeNodeBaseType_ {
  using BaseType   = TreeNodeBaseType_;
  using Scalar     = typename BaseType::Scalar;
  using VectorType = typename BaseType::VectorType;
  using MatrixType = typename BaseType::MatrixType;
  using ContainerType   = typename BaseType::ContainerType;
  using IteratorType=typename BaseType::IteratorType;
  static constexpr int Dim = BaseType::Dim_;

  TreeNodeMiddle_(IteratorType begin_,
                  IteratorType end_,
                  int num_points_,
                  int max_points_in_leaf): BaseType(begin_, end_, num_points_){
    MatrixType cov;
    computeMeanAndCovariance(_center, cov, begin_, end_);
    _direction=largestEigenVector(cov);
    IteratorType middle=split(begin_,
                              end_,
                              [&](const VectorType& v)->bool {
                                return (v-_center).dot(_direction)<0;
                              }
                              );
    // cerr << "split ("
    //      << std::distance(this->_begin, middle) << ","
    //      << std::distance(middle, this->_end) << ")";
    
    _left=BaseType::buildTree(this->_begin, middle, max_points_in_leaf);
    _right=BaseType::buildTree(middle, this->_end, max_points_in_leaf);
  }

   void search(ContainerType& answers, const VectorType& query, float squared_norm) const override {
    const Scalar distance_from_split_plane=(query-_center).transpose()*_direction;
    // if on one side, scalar>0, other side, scalar<0
    if ( distance_from_split_plane <0){
      return _left->search(answers, query, squared_norm);
    } else {
      return _right->search(answers, query, squared_norm);
    }
  }
protected:
  VectorType _direction;
  VectorType _center;
  TreeNodeBasePtr_<ContainerType> _left;
  TreeNodeBasePtr_<ContainerType> _right;
};

template <typename ContainerType_>
TreeNodeBasePtr_<ContainerType_>
TreeNodeBase_<ContainerType_>::buildTree(IteratorType begin_,
                                         IteratorType end_,
                                         int max_points_in_leaf_){
  int num_points=std::distance(begin_, end_);
  if (num_points<max_points_in_leaf_) {
    return TreeNodeBasePtr_<ContainerType_>(new TreeNodeLeaf_<BaseType>(begin_, end_, num_points));
  } else {
    return TreeNodeBasePtr_<ContainerType_>(new TreeNodeMiddle_<BaseType>(begin_,
                                                                          end_,
                                                                          num_points,
                                                                          max_points_in_leaf_));
  }
}
