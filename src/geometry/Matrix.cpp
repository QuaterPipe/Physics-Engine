#include "../include/geometry/Matrix.hpp"
namespace geo
{
	Matrix2::Matrix2() noexcept
	{
		a = 1;
		d = 1;
	}

	Matrix2::Matrix2(const f64& radians) noexcept
	{
		a = cos(radians);
		b = -sin(radians);
		c = sin(radians);
		d = cos(radians);
	}

	Matrix2::Matrix2(const f64& a, const f64& b, const f64& c, const f64& d) noexcept
	{
		this->a = a;
		this->b = b;
		this->c = c;
		this->d = d;
	}

	Matrix2& Matrix2::operator=(const Matrix2& other) noexcept
	{
		this->a = other.a;
		this->b = other.b;
		this->c = other.c;
		this->d = other.d;
		return *this;
	}

	Vector2 Matrix2::AxisX() const noexcept
	{
		return Vector2(a, c);
	}

	Vector2 Matrix2::AxisY() const noexcept
	{
		return Vector2(b, d);
	}

	f64 Matrix2::Determinant() const noexcept
	{
		return a * d - b * c;
	}

	Matrix2 Matrix2::Transpose() const noexcept
	{
		return Matrix2(a, c, b, d);
	}

	Vector2 Matrix2::operator*(const Vector2& v) const noexcept
	{
		return Vector2(a * v.x + b * v.y, c * v.x + d * v.y);
	}

	Matrix2 Matrix2::operator*(const Matrix2& other) const noexcept
	{
		Matrix2 newMatrix;
		Vector2 I = other.a * iHat;
		Vector2 II = other.c * jHat;
		Vector2 III = other.b * iHat;
		Vector2 IV = other.d * jHat;
		newMatrix.iHat = (I + II);
		newMatrix.jHat = (III + IV);
		return newMatrix;
	}

	bool Matrix2::operator==(const Matrix2& other) const noexcept
	{
		return iHat == other.iHat && jHat == other.jHat;
	}

	bool Matrix2::operator!=(const Matrix2& other) const noexcept
	{
		return jHat != other.iHat || jHat != other.jHat;
	}

	void Matrix2::Set(const f64& radians) noexcept
	{
		a = cos(radians);
		b = -sin(radians);
		c = sin(radians);
		d = cos(radians);
	}

	void Matrix2::Set(const f64& a, const f64& b, const f64& c, const f64& d) noexcept
	{
		this->a = a;
		this->b = b;
		this->c = c;
		this->d = d;
	}

	Matrix3::Matrix3() noexcept
	{
		a = 1;
		e = 1;
		i = 1;
	}

	Matrix3::Matrix3(const Matrix2& mat2) noexcept
	{
		a = mat2.a;
		b = mat2.b;
		d = mat2.c;
		e = mat2.d;
		c = 0, f = 0, g = 0, h = 0, i = 1;
		/* 
		[mat2.a, mat2.b, 0,
		 mat2.c, mat2.d, 0,
		 	0, 	0, 	1]*/
	}

	Matrix3::Matrix3(const Matrix3& mat3) noexcept
	{
		iHat = mat3.iHat;
		jHat = mat3.jHat;
		kHat = mat3.kHat;
	}

	Matrix3::Matrix3(const f64& a, const f64& d, const f64& g, const f64& b, const f64& e, const f64& h, const f64& c, const f64& f, const f64& i) noexcept
	{
		this->a = a;
		this->d = d;
		this->g = g;
		this->b = b;
		this->e = e;
		this->h = h;
		this->c = c;
		this->f = f;
		this->i = i;
	}

	Matrix3::Matrix3(const Vector3& iHat, const Vector3& jHat, const Vector3& kHat) noexcept
	{
		this->iHat = iHat;
		this->jHat = jHat;
		this->kHat = kHat;
	}

	Matrix3& Matrix3::operator=(const Matrix3& other) noexcept
	{
		this->iHat = other.iHat;
		this->jHat = other.jHat;
		this->kHat = other.kHat;
		return *this;
	}

	Vector3 Matrix3::AxisX() const noexcept
	{
		return iHat;
	}

	Vector3 Matrix3::AxisY() const noexcept
	{
		return jHat;
	}

	Vector3 Matrix3::AxisZ() const noexcept
	{
		return kHat;
	}

	f64 Matrix3::Determinant() const noexcept
	{
		return Matrix2(e, f, h, i).Determinant() - 
			Matrix2(d, f, g, i).Determinant() -
			Matrix2(d, e, g, h).Determinant();
	}

	Matrix3 Matrix3::Transpose() const noexcept
	{
		return Matrix3(a, b, c, d, e, f, g, h, i);	
	}

	bool Matrix3::operator==(const Matrix3& other) const noexcept
	{
		return iHat == other.iHat && jHat == other.jHat && kHat == other.kHat;
	}

	bool Matrix3::operator!=(const Matrix3& other) const noexcept
	{
		return iHat != other.iHat || jHat != other.jHat || kHat != other.kHat;
	}

	Vector3 Matrix3::operator*(const Vector3& other) const noexcept
	{
		return Vector3(
			a * other.x + b * other.y + c * other.z,
			d * other.x + e * other.y + f * other.z,
			g * other.x + h * other.y + i * other.z
		);
	}

	Matrix3 Matrix3::operator*(const Matrix3& other) const noexcept
	{
		Matrix3 newMatrix;
		Vector3 vecs[9];
		vecs[0] = iHat * other.a;
		vecs[1] = jHat * other.b;
		vecs[2] = kHat * other.c;
		vecs[3] = iHat * other.d;
		vecs[4] = jHat * other.e;
		vecs[5] = kHat * other.f;
		vecs[6] = iHat * other.g;
		vecs[7] = jHat * other.h;
		vecs[8] = kHat * other.i;
		newMatrix.iHat = vecs[0] + vecs[3] + vecs[6];
		newMatrix.jHat = vecs[1] + vecs[4] + vecs[7];
		newMatrix.kHat = vecs[2] + vecs[5] + vecs[8];
		return newMatrix;
	}

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
	
	Matrix::Matrix(u32 width, u32 height, f64 n) noexcept
	: _width(width), _height(height)
	{
		array = new f64[_width * _height];
		for (size_t i = 0; i < _width * _height; i++)
			array[i] = n;
	}

	Matrix::Matrix(f64* arr, u32 width, u32 height) noexcept
	: _width(width), _height(height)
	{
		array = new f64[_width * _height];
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
			x[i] = operator[](i)[index];
		return x;
	}

	i32 Matrix::GetDeterminant() const
	{
		assert(_width == _height && "Cannot find determinant of non-square matrix.");
		if (!_width)
			return 0;
		if (_width == 1)
			return (*this)[0][0];
		if (_width == 2)
			return (*this)[0][0] * (*this)[1][1] - (*this)[0][1] * (*this)[1][0];
		i32 dimension = _width;
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
						subMatrix[m - 1][z] = operator[](m)[n];
						z++;
					}
				}
			}
			result = result + sign * (*this)[0][1] * subMatrix.GetDeterminant();
			sign = -sign;
		}
		return result;
	}

	Matrix Matrix::GetTranspose() const
	{
		Matrix solution(_height, _width);
		for (size_t i = 0; i < _height; i++)
		{
			for (size_t j = 0; j < _width; j++)
				solution[j][i] = (*this)[i][j];
		}
		return solution;
	}

	const Matrix::Row Matrix::operator[](size_t index) const
	{
		assert(index < _width && "Index out of range.");
		return Row(array, index * _width, index * _width + _width);
	}

	Matrix::Row Matrix::operator[](size_t index)
	{
		assert(index < _height && "Index out of range.");
		return Row(array, index * _width, index * _width + _width);
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

	Vector Matrix::operator*(const Vector& v) const noexcept
	{
		Vector vCopy = v;
		if (vCopy.GetSize() < _height)
			vCopy.SetSize(_height);
		Vector a;
		for (u32 i = 0; i < _width; i++)
		{
			Vector x(_width);
			for (u32 j = 0; j < _width; j++)
				x[j] = (*this)[i][j];
			a[i] = (x * vCopy).Sum();
		}
		return a;
	}

	Matrix Matrix::operator*(const Matrix& other) const noexcept
	{
		u32 w = std::max(_width, other.GetWidth());
		u32 h = std::max(_height, other.GetHeight());
		f64 thisCopy[h][w] = {};
		for (u32 i = 0; i < _height; i++)
		{
			for (u32 j = 0; j < _width; j++)
				thisCopy[i][j] = (*this)[i][j];
		}
		f64 otherCopy[h][w] = {};
		for (u32 i = 0; i < other.GetHeight(); i++)
		{
			for (u32 j = 0; j < other.GetWidth(); j++)
				otherCopy[i][j] = other[i][j];
		}
		Matrix result(w, h);
		for (u32 i = 0; i < h; i++)
		{
			for (u32 k = 0; k < w; k++)
			{
				for (u32 j = 0; j < w; j++)
					result[i][j] += thisCopy[i][k] * otherCopy[k][j];
			}
		}
		return result;
	}

	size_t Matrix::GetHeight() const noexcept
	{
		return _height;
	}

	size_t Matrix::GetWidth() const noexcept
	{
		return _width;
	}

	Matrix::Row::Row(f64* data, size_t start, size_t end) noexcept
	: data(data), start(start), end(end)
	{
	}

	f64& Matrix::Row::operator[](size_t index)
	{
		size_t width = end - start;
		assert(0 <= index && index < width);
		return data[start + index];
	}

	const f64& Matrix::Row::operator[](size_t index) const
	{
		size_t width = end - start;
		assert(0 <= index && index < width);
		return data[start + index];
	}

	Matrix::Row& Matrix::Row::operator=(const Row& other)
	{
		assert(other.end - other.start == end - start);
		for (size_t i = 0; i < end - start; i++)
			data[start + i] = other[i];
		return *this;
	}
}