// Authored By: Tom Tsiliopoulos - 100616336
// Authored By: Joss Moo-Young - 100586602
// Modified By: Shawn Matthews

#pragma once
#include <vector>
#include <list>
//#include "UtilityMath.h"


namespace util
{
	template<typename T, typename R>
	inline R lerp(T from, T to, double tValue)
	{
		return (1.0 - tValue) * from + (tValue * to);
	}

	template <typename T>
	float invLerp(T d, T d0, T d1)
	{
		return (d - d0) / (d1 - d0);
	}

	//linearly remap a value val from a0 to b0 to be a value from a1 to b1. e.g. 0.3 from [0, 1] remapped to [100, 1000] becomes 370
	template <typename T>
	float lmap(T val, T a0, T b0, T a1, T b1)
	{
		float tVal = (val - a0) / (b0 - a0);
		return ((1.0f - tVal) * a1) + (b1 * tVal);
	}


	template<class T>
	struct NodeGraphTableEntry
	{
		NodeGraphTableEntry();

		NodeGraphTableEntry(T value, float a_t, float a_distanceAlongPath = 0.f);

		T val;
		float t; // normalized distance along local interval
		float distanceAlongPath; // distance along the path to this point
	};

	template<class T>
	inline NodeGraphTableEntry<T>::NodeGraphTableEntry()
	{
		val = T();
		t = 0.0f;
		distanceAlongPath = 0.0f;
	}

	template<class T>
	inline NodeGraphTableEntry<T>::NodeGraphTableEntry(T value, float a_t = 0.0f, float a_distanceAlongPath = 0.0f)
	{
		val = value;
		t = a_t;
		distanceAlongPath = a_distanceAlongPath;
	}


	template<class T>
	class Path;

	template<class T>
	Path<T> createDefaultTable();

	template<>
	Path<float> createDefaultTable(); //creates a lerp function table, i.e. it's a line that goes from 0 to 1 with a slope of 1

	template<typename T>
	inline void GenDefaultTable(std::list<NodeGraphTableEntry<T>>& list) {

	};

	template<>
	inline void GenDefaultTable<float>(std::list<NodeGraphTableEntry<float>>& list) {
		std::list<NodeGraphTableEntry<float>> intervalData = std::list<NodeGraphTableEntry<float>>();
		NodeGraphTableEntry<float> entry = NodeGraphTableEntry<float>(0.0f, 0.0f, 0.0f);
		list.push_back(entry);
		entry = NodeGraphTableEntry<float>(1.0f, 1.0f, 1.0f);
		list.push_back(entry);
	};

	//A table that defines a path, approximated into straight lines
	template<class T>
	class Path
	{
	public:
		Path();

		std::vector<std::list<NodeGraphTableEntry<T>>> m_data; // vector of lists. each list is a table of data for one interval on the path

		float getLength() const; // returns a float for length along the path. NOT table size
		size_t numIntervals() const; // returns number of intervals i.e. nodes
		void updateDistances(); //call this once if you modify the table data (NodeGrapher does this automatically)

		//linear search functions
		unsigned int lookupInterval(const float& distance);

		typename std::list<NodeGraphTableEntry<T>>::iterator iterByDist(int vectorIndex, float dist); //returns the iterator at vectorIndex with the highest distanceAlongPath value that is less than argument distanceAlongPath																		//todo: optimize iterByValue to take in or hold the last iterator so it doesnt have to restart the search every time														  //template <typename T>
		typename std::list<NodeGraphTableEntry<T>>::iterator iterByTValue(int vectorIndex, float tVal); //returns the iterator at vectorIndex with the highest t value that is less than argument tVal

		T lookupValue(const float& distance); // searches based off of distance along whole path
		T lookupPointIndexAndDistValue(int a_index, float a_distAlongPath);
		T lookupPointIndexAndTValue(int a_index, float a_tLocal);
	private:
		float m_length; // length of the path defined by this data
	};

	template<class T>
	inline Path<T>::Path()
	{
	}

	template<class T>
	inline float Path<T>::getLength() const
	{
		return m_length;
	}

	template<class T>
	inline size_t Path<T>::numIntervals() const
	{
		return m_data.size();
	}

	template<class T>
	inline void Path<T>::updateDistances()
	{
		double totalDistance = 0.0;
		// will get constantly updated. represents total distance along whole curve
		for (int interval = 0; interval < m_data.size(); interval++) // interval (the current keyNode)
		{
			// compute pairwise distances and distance along path for all points in the table
			std::list<NodeGraphTableEntry<T>>& table = m_data[interval];
			table.front().distanceAlongPath = totalDistance;
			std::list<NodeGraphTableEntry<T>>::iterator row = table.begin();

			while (std::next(row) != table.end())
			{
				T lastRow = row->val;
				++row;
				T currentRow = row->val;
				float pairwiseDist = glm::length(currentRow - lastRow);
				totalDistance += pairwiseDist;
				row->distanceAlongPath = totalDistance;
			}
		}
		m_length = (float)totalDistance;
	}

	template<class T>
	inline unsigned int Path<T>::lookupInterval(const float & distance)
	{
		unsigned int interval = m_data.size() - 1;
		for (; m_data[interval].begin()->distanceAlongPath > distance; interval--)
		{
			//haha bad
		}
		return interval;
	}

	template<class T>
	inline typename std::list<NodeGraphTableEntry<T>>::iterator Path<T>::iterByTValue(int vectorIndex, float tVal)
	{
		std::list<NodeGraphTableEntry<T>> &row = m_data[vectorIndex];
		for (auto rit = ++row.rbegin(); rit != row.rend(); rit++) // reverse iteration. rBegin() points to the same element as --end()
		{
			if (rit->t <= tVal)
			{
				//when converting from reverse iterator to forward iterator, we must first add 1 to make sure we are still pointing at the same thing
				return (++rit).base();
			}
		}
		//throw std::exception("could not find in interval!");
		return row.end(); // couldnt find it, i.e. somethings messed up.
	}

	template<class T>
	inline typename std::list<NodeGraphTableEntry<T>>::iterator Path<T>::iterByDist(int vectorIndex, float dist)
	{
		std::list<NodeGraphTableEntry<T>> &row = m_data[vectorIndex];
		for (auto rit = row.rbegin(); rit != row.rend(); rit++) // reverse iteration. rBegin() points to the same element as --end(). ++ causes rit to go backwards in the list
		{
			if (rit->distanceAlongPath <= dist)
			{
				//when converting from reverse iterator to forward iterator, we must first add 1 to make sure we are still pointing at the same thing
				return (++rit).base();
			}
		}
		//throw std::exception("could not find in interval!!"); //ok you broke it for real now if you got here
		return row.end();
	}

	template<class T>
	inline T Path<T>::lookupValue(const float & distance)
	{
		std::list<NodeGraphTableEntry<T>>::iterator iter;

		int interval = m_data.size() - 1;
		for (; m_data[interval].begin()->distanceAlongPath > distance; interval--)
		{
			//exits loop when correct interval is found
			//will attempt array access of -1 if nothing is found and thus exit 
			//is this bad programming?
#ifdef _DEBUG
			if (interval < 0)
			{
				throw std::exception("lookup could not find interval!");
			}
#endif // _DEBUG
		}
		T ret;
		iter = iterByDist(interval, distance);
		std::list<NodeGraphTableEntry<T>>::iterator iter_next = std::next(iter);
		if (iter_next == m_data[interval].end())
		{
			iter_next = m_data[(interval + 1) % numIntervals()].begin();
			ret = iter_next->val;
		}
		else
		{
			float tValue = (distance - iter->distanceAlongPath) / (iter_next->distanceAlongPath - iter->distanceAlongPath);
			ret = lerp(iter->val, iter_next->val, tValue);
		}
		return ret;
		}

	template<class T>
	inline T Path<T>::lookupPointIndexAndDistValue(int a_index, float a_distAlongPath)
	{
		std::list<NodeGraphTableEntry<T>>::iterator iter;
		std::list<NodeGraphTableEntry<T>>::iterator iter_next;
#ifdef NDEBUG 
		try
		{
#endif // NDEBUG
			iter = iterByDist(a_index, a_distAlongPath);
			iter_next = std::next(iter);
#ifdef NDEBUG
		}
		catch (std::exception e)
		{
		}
#endif // NDEBUG

		T ret;
		float tValue = invLerp(a_distAlongPath, iter->distanceAlongPath, iter_next->distanceAlongPath); // (a_distAlongPath - iter->distanceAlongPath) / (iter_next->distanceAlongPath - iter->distanceAlongPath);

		ret = lerp(iter->val, iter_next->val, tValue);
		return ret;
	}

	template<class T>
	inline T Path<T>::lookupPointIndexAndTValue(int a_index, float a_tLocal)
	{
		std::list<NodeGraphTableEntry<T>>::iterator iter;
		std::list<NodeGraphTableEntry<T>>::iterator iter_next;
#ifdef NDEBUG 
		try
		{
#endif // NDEBUG
			iter = iterByTValue(a_index, a_tLocal);
			iter_next = std::next(iter);
#ifdef NDEBUG
		}
		catch (std::exception e)
		{
		}
#endif // NDEBUG
		T ret;
		//create interpolation value from ratio between 3 tValues
		float tEvenMoreLocal = invLerp(a_tLocal, iter->t, iter_next->t); //(a_tLocal - iter->t, ) / (iter_next->t - iter->t);

		ret = lerp(iter->val, iter_next->val, tEvenMoreLocal);
		return ret;
	}

	template<>
	inline Path<float> createDefaultTable() // a straight line from 0 to 1 of slope 1. if you interpolate along this spline, you will get a lerp.
	{
		Path<float> ret = Path<float>();
		std::list<NodeGraphTableEntry<float>> intervalData = std::list<NodeGraphTableEntry<float>>();
		NodeGraphTableEntry<float> entry = NodeGraphTableEntry<float>(0.0f, 0.0f, 0.0f);
		intervalData.push_back(entry);
		entry = NodeGraphTableEntry<float>(1.0f, 1.0f, 1.0f);
		intervalData.push_back(entry);
		ret.m_data.push_back(intervalData);
		ret.updateDistances();
		return ret;
	}
	}
