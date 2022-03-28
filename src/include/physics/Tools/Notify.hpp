#pragma once
#include <vector>
#include <string>
namespace physics
{
	class Notifiable;
	class Notification;
	template <typename T>
	class Property;

	class Notification
	{
		private:
			Notification() = delete;
		public:
			long returnCode = 0;
			Notifiable* caller = NULL;
			std::vector<std::string> info;
			Notification(long returnCode, Notifiable* caller, std::vector<std::string> info);
			static Notification empty;
	};

	class Notifiable
	{
		public:
			virtual void Notify(Notification& n);
	};

	template <typename T>
	class Property : public Notifiable
	{
		protected:
			Property() = delete;
			T _value;
			Notifiable* _parent;
			bool _isConst;
		public:
			Property(Notifiable& parent) noexcept;
			T& Get();
			const T& Get() const;
			void operator=(const T& other);
			void operator=(T& other);
			explicit operator T&() const;
			operator T() const;
			explicit operator const T&() const;
			static long Accessed;
			static long AccessFailed;
			static long Edit;
			static long EditFailed;
	};

	//template <typename T>
	//T& operator=(const Property<T>& p);
	//template <typename T>
	//const T& operator=(const Property<T>& p);

	template <typename T>
	class RefProperty : public Notifiable
	{
		protected:
			RefProperty() = delete;
			T* _value = NULL;
			const T* _constValue = NULL;
			Notifiable* _parent;
			const bool _isConst;
		public:
			RefProperty(Notifiable& parent, T& value, bool isConst) noexcept;
			RefProperty(Notifiable& parent, const T& value) noexcept;
			T& Get();
			const T& Get() const;
			void operator=(const T& other);
			void operator=(T& other);
			explicit operator T&() const;
			operator T() const;
			explicit operator const T&() const;
			static long Accessed;
			static long AccessFailed;
			static long Edit;
			static long EditFailed;
	};

//template <typename T>
//T& operator=(const RefProperty<T>& p);

//template <typename T>
//const T& operator=(const RefProperty<T>& p);
}
#include "Notify.inl"