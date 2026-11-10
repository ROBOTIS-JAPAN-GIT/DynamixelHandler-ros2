#ifndef MAKE_ITERATOR_CONVENIENT_HPP
#define MAKE_ITERATOR_CONVENIENT_HPP

using std::vector;
using std::set;
using std::unordered_set;

// enum でインクリメントをするため
template<typename T> T& operator ++ (T& v     ) { v = static_cast<T>(v + 1); return v;}
template<typename T> T  operator ++ (T& v, int) { T p=v; ++v; return p;}
// vectorやsetに値が含まれているかどうかを調べる関数
template <typename S, typename T> bool is_in(const S& val, const        vector<T>& v) { return std::find(v.begin(), v.end(), val) != v.end(); }
template <typename S, typename T> bool is_in(const S& val, const           set<T>& s) { auto it=s.find(val); return it!=s.end() && *it==val; }
template <typename S, typename T> bool is_in(const S& val, const unordered_set<T>& s) { auto it=s.find(val); return it!=s.end() && *it==val; }
#endif /* MAKE_ITERATOR_CONVENIENT_HPP */