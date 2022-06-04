namespace physics
{
	template <typename T>
	Ref<T>::Ref(T& value)
	{
		reference = &value;
	}

	template <typename T>
	Ref<T>::operator T&()
	{
		return *reference;
	}

	template <typename T>
	void Ref<T>::Reset(T& value)
	{
		reference = &value;
	}

	template <typename T>
	ConstRef<T>::ConstRef(const T& value)
	{
		reference = &value;
	}

	template <typename T>
	ConstRef<T>::operator const T&() const
	{
		return *reference;
	}

	template <typename T>
	void ConstRef<T>::Reset(const T& value)
	{
		reference = &value;
	}
}