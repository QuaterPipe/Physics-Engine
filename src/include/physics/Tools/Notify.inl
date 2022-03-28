namespace physics
{
	template <typename T>
	long Property<T>::Accessed = 1;
	template <typename T>
	long Property<T>::AccessFailed = 2;
	template <typename T>
	long Property<T>::Edit = 3;
	template <typename T>
	long Property<T>::EditFailed = 4;
	template <typename T>
	long RefProperty<T>::Accessed = 1;
	template <typename T>
	long RefProperty<T>::AccessFailed = 2;
	template <typename T>
	long RefProperty<T>::Edit = 3;
	template <typename T>
	long RefProperty<T>::EditFailed = 4;

	template <typename T>
	Property<T>::Property(Notifiable& parent) noexcept
	: _parent(&parent)
	{
	}

	template <typename T>
	T& Property<T>::Get()
	{
		Notification n(Accessed, this, std::vector<std::string>());
		_parent->Notify(n);
		return _value;
	}

	template <typename T>
	const T& Property<T>::Get() const
	{
		Notification n(Accessed, this, std::vector<std::string>());
		_parent->Notify(n);
		return _value;
	}

	template <typename T>
	void Property<T>::operator=(const T& other)
	{
		try
		{
			if (!_isConst)
				_value = other;
			else
				return;
		}
		catch (...)
		{
			std::exception_ptr p = std::current_exception();
			std::vector<std::string> Error;
			Error.push_back(p ? p.__cxa_exception_type()->name() : "null");
			Notification n(AccessFailed, this, Error);
			_parent->Notify(n);
			return;
		}
		Notification n(Accessed, this, std::vector<std::string>());
		_parent->Notify(n);
	}

	template <typename T>
	void Property<T>::operator=(T& other)
	{
		try
		{
			if (!_isConst)
				_value = other;
			else
				return;
		}
		catch (...)
		{
			std::exception_ptr p = std::current_exception();
			std::vector<std::string> Error;
			Error.push_back(p ? p.__cxa_exception_type()->name() : "null");
			Notification n(AccessFailed, this, Error);
			_parent->Notify(n);
			return;
		}
		Notification n(Accessed, this, std::vector<std::string>());
		_parent->Notify(n);
	}

	template <typename T>
	Property<T>::operator T&() const
	{
		return (T&)_value;
	}

	template <typename T>
	Property<T>::operator T() const
	{
		return _value;
	}

	template <typename T>
	Property<T>::operator const T&() const
	{
		return (const T&)_value;
	}

	template <typename T>
	RefProperty<T>::RefProperty(Notifiable& parent, T& value, bool isConst) noexcept
	: _parent(&parent), _value(&value), _isConst(isConst)
	{
	}

	template <typename T>
	RefProperty<T>::RefProperty(Notifiable& parent, const T& value) noexcept
	: _parent(&parent), _constValue(&value), _isConst(true)
	{
	}

	template <typename T>
	T& RefProperty<T>::Get()
	{
		return *_value;
	}

	template <typename T>
	const T& RefProperty<T>::Get() const
	{
		return *_value;
	}

	template <typename T>
	void RefProperty<T>::operator=(const T& other)
	{
		try
		{
			if (!_isConst)
				*_value = other;
			else
				return;
		}
		catch (...)
		{
			std::exception_ptr p = std::current_exception();
			std::vector<std::string> Error;
			Error.push_back(p ? p.__cxa_exception_type()->name() : "null");
			Notification n(AccessFailed, this, Error);
			_parent->Notify(n);
			return;
		}
		Notification n(Accessed, this, std::vector<std::string>());
		_parent->Notify(n);
	}

	template <typename T>
	void RefProperty<T>::operator=(T& other)
	{
		try
		{
			if (!_isConst)
				*_value = other;
			else
				return;
		}
		catch (...)
		{
			std::exception_ptr p = std::current_exception();
			std::vector<std::string> Error;
			Error.push_back(p ? p.__cxa_exception_type()->name() : "null");
			Notification n(AccessFailed, this, Error);
			_parent->Notify(n);
			return;
		}
		
		Notification n(Accessed, this, std::vector<std::string>());
		_parent->Notify(n);
	}

	template <typename T>
	RefProperty<T>::operator T&() const
	{
		return *_value;
	}

	template <typename T>
	RefProperty<T>::operator T() const
	{
		return *_value;
	}

	template <typename T>
	RefProperty<T>::operator const T&() const
	{
		return *_value;
	}

	/*template <typename T>
	T& operator=(const RefProperty<T>& p)
	{
		return (T&)p;
	}

	template <typename T>
	const T& operator=(const RefProperty<T>& p)
	{
		return (const T&)p;
	}

	template <typename T>
	T& operator=(const Property<T>& p)
	{
		return (T&)p;
	}

	template <typename T>
	const T& operator=(const Property<T>& p)
	{
		return (const T&)p;
	}*/
}