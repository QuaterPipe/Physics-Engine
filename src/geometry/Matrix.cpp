#include "geometry/Matrix.hpp"
namespace geo
{
	Matrix::~Matrix() noexcept
	{
		delete[] array;
	}

	Matrix::Matrix() noexcept
	{
	}

	Matrix::Matrix(Matrix && mat) noexcept
	{
		array = mat.array;
		mat.array = NULL;
	}

	Matrix::Matrix(const Matrix& mat) noexcept
	{
		array = new f64[mat._height * mat._width];
		for (size_t i = 0; i < mat._width * mat._height; i++)
			array[i] = mat.array[i];
	}
	
	Matrix::Matrix(size_t width, size_t height, f64 n) noexcept
	: _width(width), _height(height)
	{
		array = new f64[_width * _height];
		for (size_t i = 0; i < _width * _height; i++)
			array[i] = n;
	}

	Matrix::Matrix(const f64* arr, size_t width, size_t height) noexcept
	: _width(width), _height(height)
	{
		array = new f64[width * height];
		for (size_t i = 0; i < width * height; i++)
			array[i] = arr[i];
	}


	f64& Matrix::operator()(size_t i, size_t j)
	{
		assert(i < _width && j < _height);
		return array[i + j * _width];
	}
	const f64& Matrix::operator()(size_t i, size_t j) const
	{
		assert(i < _width && j < _height);
		return array[i + j * _width];
	}

	Matrix& Matrix::operator=(const Matrix& other) noexcept
	{
		delete array;
		array = new f64[other.GetWidth() * other.GetHeight()];
		_width = other.GetWidth();
		_height = other.GetHeight();
		for (size_t i = 0; i < _height * _width; i++)
			array[i] = other.array[i];
		return *this;
	}

	
	Vector Matrix::Axis(u32 index) const
	{
		assert(index < _width && "Index out of range.");
		Vector x(_height, 0);
		for (size_t i = 0; i < _height; i++)
			x[i] = (*this)(i, index);
		return x;
	}

	i32 Matrix::GetDeterminant() const
	{
		assert(_width == _height && "Cannot find determinant of non-square matrix.");
		if (!_width)
			return 0;
		if (_width == 1)
			return (i32)(*this)(0, 0);
		if (_width == 2)
			return (i32)(*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
		i32 dimension = (i32)_width;
		f64 result = 0;
		i32 sign = 1;
		for (i32 i = 0; i < dimension; i++)
		{
			Matrix subMatrix(dimension - 1, dimension - 1);
			for (i32 m = 1; m < dimension; m++)
			{
				i32 z = 0;
				for (int n = 0; n < dimension; n++)
				{
					if (n != i)
					{
						subMatrix(m - 1, z) = (*this)(m, n);
						z++;
					}
				}
			}
			result = result + sign * (*this)(0, 1) * subMatrix.GetDeterminant();
			sign = -sign;
		}
		return (i32)result;
	}

	Matrix Matrix::GetTranspose() const
	{
		Matrix solution(_height, _width);
		for (size_t i = 0; i < _height; i++)
		{
			for (size_t j = 0; j < _width; j++)
				solution(j, i) = (*this)(i, j);
		}
		return solution;
	}

	bool Matrix::operator==(const Matrix& other) const noexcept
	{
		if (_width != other.GetWidth() || _height != other.GetHeight())
			return false;
		for (size_t i = 0; i < _width * _height; i++)
		{
			if (array[i] != other.array[i])
				return false;
		}
		return true;
	}
	
	bool Matrix::operator!=(const Matrix& other) const noexcept
	{
		if (_width != other.GetWidth() || _height != other.GetHeight())
			return true;
		for (size_t i = 0; i < _width * _height; i++)
		{
			if (array[i] == other.array[i])
				return false;
		}
		return true;
	}

	Vector Matrix::operator*(const Vector& v) const
	{
		assert(v.GetSize() == _height);
		Vector a;
		for (u32 i = 0; i < _width; i++)
		{
			Vector x(_height);
			for (u32 j = 0; j < _height; j++)
				x[j] = (*this)(i, j);
			a[i] = (x * v).Sum();
		}
		return a;
	}

	Matrix Matrix::operator*(const Matrix& other) const
	{
		assert(other.GetWidth() == _width && other.GetHeight() == _height);
		Matrix result(_width, _height);
		for (size_t i = 0; i < _height; i++)
		{
			for (size_t k = 0; k < _width; k++)
			{
				for (size_t j = 0; j < _width; j++)
					result(i, j) += (*this)(i, k) * other(k, j);
			}
		}
		return result;
	}

	Matrix& Matrix::operator*=(const Matrix& other)
	{
		*this = (*this) * other;
		return *this;
	}


	size_t Matrix::GetHeight() const noexcept
	{
		return _height;
	}

	size_t Matrix::GetWidth() const noexcept
	{
		return _width;
	}
}