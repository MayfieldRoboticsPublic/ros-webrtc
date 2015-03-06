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

MediaType MediaType::parse(const std::string& value) {
    MediaType parsed;
    boost::smatch m;
    if (!boost::regex_match(value, m, media_type_re)) {
        std::stringstream ss;
        ss << "Malformed media type '" << value << "'.";
        throw std::invalid_argument(ss.str());
    }
    parsed.type = m[1];
    parsed.tree = m[3];
    parsed.sub_type = m[4];
    parsed.suffix = m[6];
    if (m[8].matched) {
        std::string match = m[8];
        boost::sregex_iterator m_i(match.begin(), match.end(), media_type_param_re);
        boost::sregex_iterator m_i_done;
        while (m_i != m_i_done) {
            if ((*m_i)[3].matched)
                parsed.params[(*m_i)[1]] = (*m_i)[3];
            else
                parsed.params[(*m_i)[1]] = (*m_i)[4];
            m_i++;
        }
    }
    return parsed;
}

bool MediaType::has_tree() const {
    return !tree.empty();
}

bool MediaType::has_suffix() const {
    return !suffix.empty();
}
