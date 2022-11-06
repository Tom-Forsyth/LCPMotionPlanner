#pragma once

#include "ContactPoint.h"
#include <string>
#include <map>
#include <vector>

namespace MotionPlanner
{
	class ContactManager
	{
	private:
		/// @brief Rigid body name and contact point pairs before processing.
		std::vector<std::pair<std::string, ContactPoint>> m_rawContacts;

		/// @brief Rigid body name and contact point pairs after processing.
		std::map<std::string, ContactPoint> m_processedContacts;

		/// @brief Estimated upper bound on number of contacts to reserve memory.
		const size_t m_maxNumContacts = 50;

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