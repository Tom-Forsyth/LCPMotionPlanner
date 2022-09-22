#pragma once

#include "ContactPoint.h"
#include <string>
#include <map>

namespace CollisionAvoidance
{
	class ContactManager
	{
	private:
		std::map<std::string, ContactPoint> m_contacts;

	public:
		// Constructor.
		ContactManager();

		// Destructor.
		~ContactManager();

		// Add a contact to the manager.
		void addContact(const std::string& colliderName, const ContactPoint& contactPoint);

		// Clear the contact point map.
		void clearContacts();

		// Process contacts.
		void processContacts();

	};
}