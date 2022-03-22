/**
 * @file mtk/src/vectview.hpp
 * @brief Wrapper class around a pointer used as interface for plain vectors.
 */

#ifndef VECTVIEW_HPP_
#define VECTVIEW_HPP_

#include <Eigen/Core>

namespace MTK {

/**
 * A view to a vector.
 * Essentially, @c vectview is only a pointer to @c scalar but can be used directly in @c Eigen expressions.
 * The dimension of the vector is given as template parameter and type-checked when used in expressions.
 * Data has to be modifiable.
 * 
 * @tparam scalar Scalar type of the vector. 向量元素标量类型
 * @tparam dim    Dimension of the vector. 向量纬度
 * 
 * @todo @c vectview can be replaced by simple inheritance of @c Eigen::Map, as soon as they get const-correct
 */
namespace internal {
	template<class Base, class T1, class T2>
	struct CovBlock {
		typedef typename Eigen::Block<Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>, T1::DOF, T2::DOF> Type;
		typedef typename Eigen::Block<const Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>, T1::DOF, T2::DOF> ConstType;
	};

	template<class Base, class T1, class T2>
	struct CovBlock_ {
		typedef typename Eigen::Block<Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM>, T1::DIM, T2::DIM> Type;
		typedef typename Eigen::Block<const Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM>, T1::DIM, T2::DIM> ConstType;
	};

	template<typename Base1, typename Base2, typename T1, typename T2>
	struct CrossCovBlock {
		typedef typename Eigen::Block<Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF>, T1::DOF, T2::DOF> Type;
		typedef typename Eigen::Block<const Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF>, T1::DOF, T2::DOF> ConstType;
	};

	template<typename Base1, typename Base2, typename T1, typename T2>
	struct CrossCovBlock_ {
		typedef typename Eigen::Block<Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM>, T1::DIM, T2::DIM> Type;
		typedef typename Eigen::Block<const Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM>, T1::DIM, T2::DIM> ConstType;
	};

	template<class scalar, int dim>
	struct VectviewBase {
		typedef Eigen::Matrix<scalar, dim, 1> matrix_type;
		typedef typename matrix_type::MapType Type;
		typedef typename matrix_type::ConstMapType ConstType;
	};

	template<class T>
	struct UnalignedType {
		typedef T type;
	};
}

template<class scalar, int dim>
class vectview : public internal::VectviewBase<scalar, dim>::Type {
	typedef internal::VectviewBase<scalar, dim> VectviewBase;
public:
	//! plain matrix type
	typedef typename VectviewBase::matrix_type matrix_type;
	//! base type
	typedef typename VectviewBase::Type base;
	//! construct from pointer
	explicit
	vectview(scalar* data, int dim_=dim) : base(data, dim_) {}
	//! construct from plain matrix
	vectview(matrix_type& m) : base(m.data(), m.size()) {}
	//! construct from another @c vectview
	vectview(const vectview &v) : base(v) {}
	//! construct from Eigen::Block:
	template<class Base>
	vectview(Eigen::VectorBlock<Base, dim> block) : base(&block.coeffRef(0), block.size()) {}
	template<class Base, bool PacketAccess>
	vectview(Eigen::Block<Base, dim, 1, PacketAccess> block) : base(&block.coeffRef(0), block.size()) {}

	//! inherit assignment operator
	using base::operator=;
	//! data pointer
	scalar* data() {return const_cast<scalar*>(base::data());}
};

/**
 * @c const version of @c vectview.
 * Compared to @c Eigen::Map this implementation is const correct, i.e.,
 * data will not be modifiable using this view.
 * 
 * @tparam scalar Scalar type of the vector.
 * @tparam dim    Dimension of the vector.
 * 
 * @sa vectview
 */
template<class scalar, int dim>
class vectview<const scalar, dim> : public internal::VectviewBase<scalar, dim>::ConstType {
	typedef internal::VectviewBase<scalar, dim> VectviewBase;
public:
	//! plain matrix type
	typedef typename VectviewBase::matrix_type matrix_type;
	//! base type
	typedef typename VectviewBase::ConstType base;
	//! construct from const pointer
	explicit
	vectview(const scalar* data, int dim_ = dim) : base(data, dim_) {}
	//! construct from column vector
	template<int options>
	vectview(const Eigen::Matrix<scalar, dim, 1, options>& m) : base(m.data()) {}
	//! construct from row vector
	template<int options, int phony>
	vectview(const Eigen::Matrix<scalar, 1, dim, options, phony>& m) : base(m.data()) {}
	//! construct from another @c vectview
	vectview(vectview<scalar, dim> x) : base(x.data()) {}
	//! construct from base
	vectview(const base &x) : base(x) {}
	/**
	 * Construct from Block
	 * @todo adapt this, when Block gets const-correct
	 */
	template<class Base>
	vectview(Eigen::VectorBlock<Base, dim> block) : base(&block.coeffRef(0)) {}
	template<class Base, bool PacketAccess>
	vectview(Eigen::Block<Base, dim, 1, PacketAccess> block) : base(&block.coeffRef(0)) {}

};


} // namespace MTK

#endif /* VECTVIEW_HPP_ */
