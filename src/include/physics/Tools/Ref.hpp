namespace physics
{
	template <typename T>
	struct Ref
	{
		protected:
			T* reference;
			Ref() = delete;
			Ref& operator=(const Ref& other) = delete;
			Ref& operator=(Ref&& other) = delete;
		public:
			Ref(T& value);
			operator T&();
			void Reset(T& value);
	};

	template <typename T>
	struct ConstRef
	{
		protected:
			const T* reference;
			ConstRef() = delete;
		public:
			ConstRef(const T& value);
			operator const T&() const;
			void Reset(const T& value);
	};
}
#include "Ref.inl"