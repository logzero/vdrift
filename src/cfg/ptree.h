#ifndef _PTREE_H
#define _PTREE_H

#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

class PTree;

// stream operator for a vector of values
template <typename T>
std::istream & operator>>(std::istream & stream, std::vector<T> & out)
{
	if (out.size() > 0)
	{
		// set vector
		for (size_t i = 0; i < out.size() && !stream.eof(); ++i)
		{
			std::string str;
			std::getline(stream, str, ',');
			std::stringstream s(str);
			s >> out[i];
		}
	}
	else
	{
		// fill vector
		while (stream.good())
		{
			std::string str;
			std::getline(stream, str, ',');
			std::stringstream s(str);
			T value;
			s >> value;
			out.push_back(value);
		}
	}
  return stream;
}

// include callback, called when "include" property name is encountered
struct Include
{
	virtual void operator()(PTree & node, std::string & value) = 0;
};

/*
# ini format
key1 = value1

[key2.key3]
key4 = value4

[key2]
key5 = value5
*/
void read_ini(std::istream & in, PTree & p, Include * inc = 0);
void write_ini(const PTree & p, std::ostream & out);

/*
; inf format
key1 value1
key2
{
    key3
    {
        key4 value4
    }
    key5 value5
}
*/
void read_inf(std::istream & in, PTree & p, Include * inc = 0);
void write_inf(const PTree & p, std::ostream & out);

/*
<!-- xml format -->
<key1>value1</key1>
<key2>
    <key3>
		<key4>value4</key4>
	</key3>
    <key5>value5</key5>
</key2>
*/
void read_xml(std::istream & in, PTree & p, Include * inc = 0);
void write_xml(const PTree & p, std::ostream & out);


class PTree
{
public:
	typedef std::map<std::string, PTree> map;
	typedef map::const_iterator const_iterator;
	typedef map::iterator iterator;

	PTree() : _parent(0) {}

	PTree(const std::string & value) : _value(value), _parent(0) {}

	const_iterator begin() const
	{
		return _children.begin();
	}

	const_iterator end() const
	{
		return _children.end();
	}

	int size() const
	{
		return _children.size();
	}

	const std::string & value() const
	{
		return _value;
	}

	std::string & value()
	{
		return _value;
	}

	const PTree * parent() const
	{
		return _parent;
	}

	template <typename T> bool get(const std::string & key, T & value) const
	{
		size_t next = key.find(".");
		const_iterator i = _children.find(key.substr(0, next));
		if (i != _children.end())
		{
			if (next >= key.length()-1)
			{
				_get(i->second, value);
				return true;
			}
			return i->second.get(key.substr(next+1), value);
		}
		return false;
	}

	template <typename T> bool get(const std::string & key, T & value, std::ostream & error) const
	{
		if (get(key, value))
		{
			return true;
		}

		error << fullname(key) << " not found." << std::endl;
		return false;
	}

	template <typename T> PTree & set(const std::string & key, const T & value)
	{
		size_t next = key.find(".");
		iterator i = _children.insert(std::make_pair(key.substr(0, next), PTree())).first;
		PTree & p = i->second;
		p._parent = this; // store parent pointer for error reporting
		if (next >= key.length()-1)
		{
			std::ostringstream s;
			s << value;
			p._value = s.str();
			return p;
		}
		p._value = i->first; // store node key for error reporting
		return p.set(key.substr(next), value);
	}

	// copy value, children
	void set(PTree & other)
	{
		_value = other._value;
		_children = other._children;
	}

	void clear()
	{
		_children.clear();
	}

	std::string fullname(const std::string & name = std::string()) const
	{
		std::string full_name;
		if (!name.empty())
		{
			full_name = '.' + name;
		}
		
		const PTree * node = this;
		while (node && !node->_value.empty())
		{
			full_name = '.' + node->_value + full_name;
			node = node->_parent;
		}
		
		return full_name;
	}

private:
	std::string _value;
	map _children;
	const PTree * _parent;

	template <typename T> void _get(const PTree & p, T & value) const
	{
		std::stringstream s(p._value);
		s >> value;
	}
};

// specializations
template <> inline void PTree::_get<std::string>(const PTree & p, std::string & value) const
{
	value = p._value;
}

template <> inline void PTree::_get<bool>(const PTree & p, bool & value) const
{
	value = (p._value == "1" || p._value == "true" || p._value == "on");
}

template <> inline void PTree::_get<const PTree *>(const PTree & p, const PTree * & value) const
{
	value = &p;
}

template <> inline PTree & PTree::set(const std::string & key, const PTree & value)
{
	std::string::const_iterator next = std::find(key.begin(), key.end(), '.');
	std::pair<iterator, bool> pi = _children.insert(std::make_pair(std::string(key.begin(), next), value));
	PTree & p = pi.first->second;
	if (pi.second && p._value.empty())
	{
		p._value = pi.first->first;
		p._parent = this;
	}
	if (next == key.end())
	{
		return p;
	}
	return p.set(std::string(next+1, key.end()), PTree());
}

#endif //_PTREE_H
