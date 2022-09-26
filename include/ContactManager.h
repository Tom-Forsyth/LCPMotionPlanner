#pragma once

#include "ContactPoint.h"
#include <string>
#include <map>

namespace CollisionAvoidance
{
	class ContactManager
	{
	private:
		// Link name & contact point pairs.
		std::map<std::string, ContactPoint> m_contacts;

	public:
		// Constructor.
		ContactManager();

		// Destructor.
		~ContactManager();

		// Add a contact to the manager.
		void addContact(const std::string& linkName, const ContactPoint& contactPoint);

		// Clear the contact point map.
		void clearContacts();

		// Reduce contacts to only one contact per link.
		void reduceContacts();

		// Get the map of contacts.
		const std::map<std::string, ContactPoint>& getContacts() const;

	};
}