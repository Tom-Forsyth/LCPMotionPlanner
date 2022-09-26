#include "ContactManager.h"
#include "ContactPoint.h"
#include <string>
#include <map>
#include <vector>

namespace CollisionAvoidance
{
	ContactManager::ContactManager()
	{
	
	}

	ContactManager::~ContactManager()
	{

	}

	void ContactManager::addContact(const std::string& linkName, const ContactPoint& contactPoint)
	{
		m_contacts.emplace(linkName, contactPoint);
	}

	void ContactManager::clearContacts()
	{
		m_contacts.clear();
	}

	void ContactManager::reduceContacts()
	{
		std::string previousLink = "None";
		double previousDistance = 1e10;
		std::vector<std::string> contactsToErase;
		for (const std::pair<std::string, ContactPoint>& contact : m_contacts)
		{
			// If there are multiple contacts for a link, keep only the closest contact.
			bool updatePrevious = true;
			if (contact.first == previousLink)
			{
				// Remove contact with larger distance.
				if (contact.second.m_distance > previousDistance)
				{
					contactsToErase.emplace_back(contact.first);
					updatePrevious = false;
				}
				else
				{
					contactsToErase.emplace_back(previousLink);
				}
			}

			// If we did not remove the current contact point, mark as previous.
			if (updatePrevious)
			{
				previousLink = contact.first;
				previousDistance = contact.second.m_distance;
			}
		}

		// Erase contacts that were marked for removal.
		for (const std::string& contactName : contactsToErase)
		{
			m_contacts.erase(contactName);
		}
	}

	const std::map<std::string, ContactPoint>& ContactManager::getContacts() const
	{
		return m_contacts;
	}
}