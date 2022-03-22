// This is an advanced implementation of the algorithm described in the
// following paper:
//    C. Hertzberg,  R.  Wagner,  U.  Frese,  and  L.  Schroder.  Integratinggeneric   sensor   fusion   algorithms   with   sound   state   representationsthrough  encapsulation  of  manifolds.
//    CoRR,  vol.  abs/1107.1119,  2011.[Online]. Available: http://arxiv.org/abs/1107.1119

/**
 * @file mtk/src/SubManifold.hpp
 * @brief Defines the SubManifold class
 */


#ifndef SUBMANIFOLD_HPP_
#define SUBMANIFOLD_HPP_


#include "vectview.hpp"


namespace MTK {

/**
 * @ingroup SubManifolds
 * Helper class for compound manifolds. 
 * This class wraps a manifold T and provides an enum IDX refering to the 
 * index of the SubManifold within the compound manifold. 
 *  
 * Memberpointers to a submanifold can be used for @ref SubManifolds "functions accessing submanifolds".
 * 
 * @tparam T   The manifold type of the sub-type
 * @tparam idx The index of the sub-type within the compound manifold
 */
template<class T, int idx, int dim>
struct SubManifold : public T 
{
	enum {IDX = idx, DIM = dim /*!< index of the sub-type within the compound manifold */ };
	//! manifold type
	typedef T type;
	
	//! Construct from derived type
	template<class X>
	explicit
	SubManifold(const X& t) : T(t) {};
	
	//! Construct from internal type
	//explicit
	SubManifold(const T& t) : T(t) {};
	
	//! inherit assignment operator
	using T::operator=;
	
};

}  // namespace MTK


#endif /* SUBMANIFOLD_HPP_ */
