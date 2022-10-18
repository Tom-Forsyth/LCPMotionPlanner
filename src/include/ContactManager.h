#pragma once

#include "ContactPoint.h"
#include <string>
#include <map>

namespace MotionPlanner
{
	class ContactManager
	{
	private:
		/// @brief Rigid body name and contact point pairs.
		std::map<std::string, ContactPoint> m_contacts;

	public:
		/// @brief Constructor.
		ContactManager();

		/// @brief Destructor.
		~ContactManager();

		/// @brief Add a contact point to the manager.
		/// @param linkName Name of rigid body on which this contact point is occuring.
		/// @param contactPoint Position, normal, separation, and status of contact.
		void addContact(const std::string& linkName, const ContactPoint& contactPoint);

		/// @brief Clear the contacts from the map.
		void clearContacts();

		/// @brief Reduce number of contacts to at most one per link.
		void reduceContacts();

		/// @brief Get the map of the contacts and links.
		/// @return Contact and link map.
		const std::map<std::string, ContactPoint>& getContacts() const;

	};
}