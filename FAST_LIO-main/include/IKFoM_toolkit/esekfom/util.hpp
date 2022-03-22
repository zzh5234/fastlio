

#ifndef __MEKFOM_UTIL_HPP__
#define __MEKFOM_UTIL_HPP__

#include <Eigen/Core>
#include "../mtk/src/mtkmath.hpp"
namespace esekfom {

template <typename T1, typename T2>
class is_same {
public:
    operator bool() {
        return false;
    }
};
template<typename T1>
class is_same<T1, T1> {
public:
    operator bool() {
        return true;
    }
};

template <typename T>
class is_double {
public:
    operator bool() {
        return false;
    }
};

template<>
class is_double<double> {
public:
    operator bool() {
        return true;
    }
};

template<typename T>
static T
id(const T &x)
{
	return x;
}

} // namespace esekfom
	
#endif // __MEKFOM_UTIL_HPP__
