#ifndef QUICKDEVCPP_QUICKDEV_MATRIX_H_
#define QUICKDEVCPP_QUICKDEV_MATRIX_H_

#include <quickdev/macros.h>
#include <quickdev/vector.h>
#include <iostream>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

namespace matrix_base
{
namespace policies
{
    struct Rectangular{};
    struct Square{};
    struct UpperSymmetric{};
    struct LowerSymmetric{};
    typedef UpperSymmetric Symmetric;
}
template<size_t __Rows__, size_t __Cols__, class __Policy>
struct dims{};

template<size_t __Rows__, size_t __Cols__>
struct dims<__Rows__, __Cols__, policies::Rectangular>
{
    static const size_t _DIM_ = __Rows__ * __Cols__;
};

template<size_t __Dim__>
struct dims<__Dim__, __Dim__, policies::UpperSymmetric>
{
    static const size_t _DIM_ = (__Dim__ * __Dim__ + __Dim__) / 2;
};

template<size_t __Dim__>
struct dims<__Dim__, __Dim__, policies::LowerSymmetric>
{
    static const size_t _DIM_ = (__Dim__ * __Dim__ + __Dim__) / 2;
};

}

/*template<size_t __Dim__, class __Data>
class VectorAdapter : public VectorWrapper<__Dim__, __Data> >
{
    //
};*/

template<size_t __Rows__, size_t __Cols__, class __Data, class __Policy>
class Matrix;

template<class __Matrix>
class MatrixHelper;

template<template<size_t, size_t, typename, typename> class __MatrixBase, size_t __Rows__, size_t __Cols__, class __Data, class __Policy>
class MatrixHelper<__MatrixBase<__Rows__, __Cols__, __Data, __Policy> > : public VectorWrapper<matrix_base::dims<__Rows__, __Cols__, __Policy>::_DIM_, __Data, __MatrixBase<__Rows__, __Cols__, __Data, __Policy> >
{
public:
    typedef __MatrixBase<__Rows__, __Cols__, __Data, __Policy> _MatrixBase;
    typedef Matrix<__Rows__, __Cols__, __Data, __Policy> _Matrix;

    virtual __Data & get( size_t const & row, size_t const & col ) = 0;
    virtual __Data const & cget( size_t const & row, size_t const & col ) const = 0;

    __Data & at( size_t const & row, size_t const & col )
    {
        std::cout << "got ref" << std::endl;
        return get( row, col );
    }

    __Data const & at( size_t const & row, size_t const & col ) const
    {
        std::cout << "got const ref" << std::endl;
        return cget( row, col );
    }

    __Data & operator()( size_t const & row, size_t const & col )
    {
        return get( row, col );
    }

    __Data const & operator()( size_t const & row, size_t const & col ) const
    {
        return cget( row, col );
    }

    bool isLinked( size_t const & row, size_t const & col ) const
    {
        return false;
    }

    template
    <
        class __OtherPolicy,
        typename std::enable_if<(std::is_same<__Policy, __OtherPolicy>::value), int>::type = 0
    >
    __MatrixBase<__Rows__, __Cols__, __Data, __Policy> convertTo() const
    {
        return *(__MatrixBase<__Rows__, __Cols__, __Data, __Policy>*)this;
    }
};

template<size_t __Rows__, size_t __Cols__, class __Data, class __Policy>
class MatrixBase;

//MatrixBase<__Rows__, __Cols__, __Data, __OtherPolicy>

template<size_t __Rows__, size_t __Cols__, class __Data>
class MatrixBase<__Rows__, __Cols__, __Data, matrix_base::policies::Rectangular> : public MatrixHelper<MatrixBase<__Rows__, __Cols__, __Data, matrix_base::policies::Rectangular> >
{
public:
    typedef Vector<matrix_base::dims<__Rows__, __Cols__, matrix_base::policies::Rectangular>::_DIM_, __Data> _Storage;
    typedef matrix_base::policies::Rectangular _Policy;
    typedef MatrixHelper<MatrixBase<__Rows__, __Cols__, __Data, _Policy> > _ParentMat;

    //static size_t const _DIM_ = __Rows__ * __Cols__;

    __Data & get( size_t const & row, size_t const & col )
    {
        return _Storage::at( row * __Cols__ + col );
    }

    __Data const & cget( size_t const & row, size_t const & col ) const
    {
        return _Storage::at( row * __Cols__ + col );
    }

    template
    <
        class __OtherPolicy,
        typename std::enable_if<(std::is_same<__OtherPolicy, matrix_base::policies::UpperSymmetric>::value), int>::type = 0
    >
    MatrixBase<__Rows__, __Cols__, __Data, __OtherPolicy> convertTo() const
    {
        MatrixBase<__Rows__, __Cols__, __Data, __OtherPolicy> result;
        for( size_t row = 0; row < __Rows__; ++row )
        {
            for( size_t col = 0; col < __Cols__; ++col )
            {
                if( !result.isLinked( row, col ) ) result( row, col ) = cget( row, col );
            }
        }
        return result;
    }

    template
    <
        class __OtherPolicy,
        typename std::enable_if<(std::is_same<__OtherPolicy, _Policy>::value), int>::type = 0
    >
    MatrixBase<__Rows__, __Cols__, __Data, __OtherPolicy> convertTo() const { return _ParentMat::template convertTo<__OtherPolicy>(); }
};

template<size_t __Dim__, class __Data>
class MatrixBase<__Dim__, __Dim__, __Data, matrix_base::policies::UpperSymmetric> : public MatrixHelper<MatrixBase<__Dim__, __Dim__, __Data, matrix_base::policies::UpperSymmetric> >
{
public:
    typedef Vector<matrix_base::dims<__Dim__, __Dim__, matrix_base::policies::UpperSymmetric>::_DIM_, __Data> _Storage;

    //static size_t const _DIM_ = (__Dim__ * __Dim__ + __Dim__) / 2;

    __Data & get( size_t const & row, size_t const & col )
    {
        //std::cout << "getting element [" << row << "," << col << "]" << std::endl;
        size_t const sym_row = std::min( row, col );
        size_t const sym_row_offset = sym_row + 1;
        size_t const sym_col = std::max( row, col );

        return _Storage::at( sym_row * __Dim__ - ( sym_row_offset * sym_row_offset - sym_row_offset ) / 2 + sym_col );
    }

    __Data const & cget( size_t const & row, size_t const & col ) const
    {
        size_t const sym_row = std::min( row, col );
        size_t const sym_row_offset = sym_row + 1;
        size_t const sym_col = std::max( row, col );

        return _Storage::at( sym_row * __Dim__ - ( sym_row_offset * sym_row_offset - sym_row_offset ) / 2 + sym_col );
    }

    bool isLinked( size_t const & row, size_t const & col ) const
    {
        return row > col;
    }
};

template<size_t __Dim__, class __Data>
class MatrixBase<__Dim__, __Dim__, __Data, matrix_base::policies::LowerSymmetric> : public MatrixHelper<MatrixBase<__Dim__, __Dim__, __Data, matrix_base::policies::LowerSymmetric> >
{
public:
    typedef Vector<matrix_base::dims<__Dim__, __Dim__, matrix_base::policies::LowerSymmetric>::_DIM_, __Data> _Storage;

    //static size_t const _DIM_ = (__Dim__ * __Dim__ + __Dim__) / 2;

    __Data & get( size_t const & row, size_t const & col )
    {
        size_t const sym_row = std::max( row, col );
        size_t const sym_row_offset = sym_row + 1;
        size_t const sym_col = std::min( row, col );

        return _Storage::at( sym_row * __Dim__ - ( sym_row_offset * sym_row_offset - sym_row_offset ) / 2 + sym_col );
    }

    __Data const & cget( size_t const & row, size_t const & col ) const
    {
        size_t const sym_row = std::max( row, col );
        size_t const sym_row_offset = sym_row + 1;
        size_t const sym_col = std::min( row, col );

        return _Storage::at( sym_row * __Dim__ - ( sym_row_offset * sym_row_offset - sym_row_offset ) / 2 + sym_col );
    }

    bool isLinked( size_t const & row, size_t const & col ) const
    {
        return row < col;
    }
};

template<size_t __Rows__, size_t __Cols__, class __Data, class __Policy = matrix_base::policies::Rectangular>
class Matrix : public MatrixBase<__Rows__, __Cols__, __Data, __Policy>
{
public:
    template<class __OtherPolicy>
    Matrix<__Rows__, __Cols__, __Data, __OtherPolicy> convertTo() const
    {
        auto converted_matrix = MatrixBase<__Rows__, __Cols__, __Data, __Policy>::template convertTo<__OtherPolicy>();
        return *(Matrix<__Rows__, __Cols__, __Data, __OtherPolicy>*) &converted_matrix;
        //return this->convertTo<__Other>();
        //return MatrixBase<__Rows__, __Cols__, __Data, __Policy>::template convertTo<__Other>();
    }
    //
};

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_MATRIX_H_
