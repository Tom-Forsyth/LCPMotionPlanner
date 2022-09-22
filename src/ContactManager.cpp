#include "ContactManager.h"
#include "ContactPoint.h"
#include <string>
#include <map>

namespace CollisionAvoidance
{
	ContactManager::ContactManager()
	{
	
	}

	ContactManager::~ContactManager()
	{

	}

	void ContactManager::addContact(const std::string& colliderName, const ContactPoint& contactPoint)
	{
		m_contacts.emplace(colliderName, contactPoint);
	}

	void ContactManager::clearContacts()
	{
		m_contacts.clear();
	}

	void ContactManager::processContacts()
	{

	}
}