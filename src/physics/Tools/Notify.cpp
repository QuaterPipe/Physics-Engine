#include "../../include/physics/Tools/Notify.hpp"

namespace physics
{
	Notification::Notification(long returnCode, Notifiable* caller, std::vector<std::string> info)
	: returnCode(returnCode), caller(caller), info(info)
	{
	}

	void Notifiable::Notify(Notification& n)
	{
	}
}