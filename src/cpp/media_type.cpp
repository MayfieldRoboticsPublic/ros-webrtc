#include "media_type.h"

#include <sstream>

#include <boost/regex.hpp>

/**
 * A regex used to parse media types, which are used to describe data channel
 * protocols.
 */
static const boost::regex media_type_re(
    "^"
    "(\\w+)"
    "/"
    "((vnd|prs|x)\\.)?"
    "([\\w\\.\\-]+)"
    "(\\+(\\w+))?"
    "(\\s*;((\\s*[\\w\\-_\\.]+?=(\"([^\"]+?)\"|([^\\s]+)))*))?"
    "$"
);

/**
 * A regex used to parse media type parameters.
 */
static const boost::regex media_type_param_re(
    "([\\w\\-_\\.]+?)"
    "="
    "(\"([^\"]+?)\"|([^\\s]+))"
);

bool MediaType::matches(const std::string& value) {
    return boost::regex_match(value, media_type_re);
}

MediaType::MediaType() {
}

MediaType::MediaType(const std::string& value) {
    boost::smatch m;
    if (!boost::regex_match(value, m, media_type_re)) {
        std::stringstream ss;
        ss << "Malformed media type '" << value << "'.";
        throw std::invalid_argument(ss.str());
    }
    type = m[1];
    tree = m[3];
    sub_type = m[4];
    suffix = m[6];
    if (m[8].matched) {
        std::string match = m[8];
        boost::sregex_iterator m_i(match.begin(), match.end(), media_type_param_re);
        boost::sregex_iterator m_i_done;
        while (m_i != m_i_done) {
            if ((*m_i)[3].matched)
                params[(*m_i)[1]] = (*m_i)[3];
            else
                params[(*m_i)[1]] = (*m_i)[4];
            m_i++;
        }
    }
}

bool MediaType::has_tree() const {
    return !tree.empty();
}

bool MediaType::has_suffix() const {
    return !suffix.empty();
}
