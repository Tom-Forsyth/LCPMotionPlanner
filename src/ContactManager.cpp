#include "ContactManager.h"
#include "ContactPoint.h"
#include <string>
#include <map>
#include <vector>

namespace MotionPlanner
{
	ContactManager::ContactManager()
	{
		m_rawContacts.reserve(m_maxNumContacts);
	}

	ContactManager::~ContactManager()
	{

	}

	void ContactManager::addContact(const std::string& linkName, const ContactPoint& contactPoint)
	{
		m_rawContacts.emplace_back(std::pair<std::string, ContactPoint>(linkName, contactPoint));
	}

	void ContactManager::clearContacts()
	{
		// Clear containers and reserve memory for next iteration.
		m_rawContacts.clear();
		m_processedContacts.clear();
		m_rawContacts.reserve(m_maxNumContacts);
	}

	void ContactManager::reduceContacts()
	{	
		for (const std::pair<std::string, ContactPoint>& contact : m_rawContacts)
		{
			// If the link does not have an existing contact, add it to the contact map, else compare the two.
			if (m_processedContacts.find(contact.first) == m_processedContacts.end())
			{
				// Add to the contact map.
				m_processedContacts.emplace(contact.first, contact.second);
			}
			else
			{
				// Compare the contacts, and keep the one with the smallest distance.
				const double currentDistance = m_processedContacts.at(contact.first).m_distance;
				if (currentDistance > contact.second.m_distance)
				{
					m_processedContacts.at(contact.first) = contact.second;
				}
			}
		}
	}

	const std::map<std::string, ContactPoint>& ContactManager::getContacts() const
	{
		return m_processedContacts;
	}
}