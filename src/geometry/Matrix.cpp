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
		c = 1, f = 1, g = 0, h = 0, i = 1;
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

	Matrix::Matrix() noexcept
	{
	}

	Matrix::Matrix(const Matrix& mat) noexcept
	: array(mat.array)
	{
	}
	
	Matrix::Matrix(u32 width, u32 height) noexcept
	: width(width), height(height)
	{
		array.resize(height);
		for (auto& arr: array)
			arr.resize(width);
	}

	Matrix::Matrix(f64* arr, u32 width, u32 height) noexcept
	: width(width), height(height)
	{
		array.resize(height);
		for (u32 i = 0; i < width; i++)
			array[i].resize(width);
		u32 index = 0;
		for (u32 i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
				array[i][j] = arr[index++];
		}
		
	}
	
	Vector Matrix::Axis(u32 index) const
	{
		assert(index < width && "Index out of range.");
		Vector x(height, 0);
		x.Set(array[index]);
		return x;
	}

	i32 Matrix::GetDeterminant() const
	{
		assert(width == height && "Cannot find determinant of non-square matrix.");
		if (!width)
			return 0;
		if (width == 1)
			return (*this)[0][0];
		if (width == 2)
			return (*this)[0][0] * (*this)[1][1] - (*this)[0][1] * (*this)[1][0];
		i32 dimension = width;
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
						subMatrix[m - 1][z] = array[m][n];
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
		Matrix solution(height, width);
		for (size_t i = 0; i < height; i++)
		{
			for (size_t j = 0; j < width; j++)
				solution[j][i] = (*this)[i][j];
		}
		return solution;
	}

	const std::vector<f64>& Matrix::operator[](size_t index) const
	{
		assert(index < width && "Index out of range.");
		return array[index];
	}

	std::vector<f64>& Matrix::operator[](size_t index)
	{
		assert(index < height && "Index out of range.");
		return array[index];
	}

	bool Matrix::operator==(const Matrix& other) const noexcept
	{
		if (width != other.width || height != other.height)
			return false;
		return array == other.array;
	}
	
	bool Matrix::operator!=(const Matrix& other) const noexcept
	{
		if (width != other.width || height != other.height)
			return true;
		return array != other.array;
	}

	Vector Matrix::operator*(const Vector& v) const noexcept
	{
		Vector vCopy = v;
		if (vCopy.GetSize() < height)
			vCopy.SetSize(height);
		Vector a;
		for (u32 i = 0; i < width; i++)
		{
			Vector x;
			x.Set(array[i]);
			a[i] = (x * vCopy).Sum();
		}
		return a;
	}

	Matrix Matrix::operator*(const Matrix& other) const noexcept
	{
		u32 w = std::max(width, other.width);
		u32 h = std::max(height, other.height);
		f64  x[h][w] = {};
		f64 thisCopy[h][w] = {};
		for (u32 i = 0; i < height; i++)
		{
			for (u32 j = 0; j < width; j++)
				thisCopy[i][j] = (*this)[i][j];
		}
		f64 otherCopy[h][w] = {};
		for (u32 i = 0; i < other.height; i++)
		{
			for (u32 j = 0; j < other.width; j++)
				otherCopy[i][j] = other[i][j];
		}
		for (u32 i = 0; i < h; i++)
		{
			for (u32 j = 0; j < w; j++)
			{
				for (u32 k = 0; k < w; k++)
					x[i][j] += thisCopy[i][k] * otherCopy[k][j];
			}
		}
		Matrix result(w, h);
		for (u32 i = 0; i < h; i++)
		{
			for (u32 j = 0; j < w; j++)
				result[i][j] = x[i][j];
		}
		return result;
	}

	u32 Matrix::GetHeight() const noexcept
	{
		return height;
	}

	u32 Matrix::GetWidth() const noexcept
	{
		return width;
	}

	void Matrix::SetHeight(u32 height) noexcept
	{
		array.resize(height);
		this->height = height;
		for (auto& arr: array)
		{
			if (arr.size() != width)
				arr.resize(width);
		}
	}

	void Matrix::SetWidth(u32 width) noexcept
	{
		for (auto& arr: array)
			arr.resize(width);
		this->width = width;
	}
}