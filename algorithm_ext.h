#ifndef ALGORITHM_EXT_H
#define ALGORITHM_EXT_H

#include <point_2d.h>

#include <algorithm>
#include <deque>
#include <functional>
#include <list>
#include <map>
#include <numeric>

// Example usage:
// struct Person { int age; };
// vector<Person> persons;
// sort_desc(persons, age);
// vector<Person*> persons2;
// sort_ptr_asc(persons2, age);
//
// Also works with functions instead of member variables and all containers
// that can be sorted with std::sort.
#define sort_desc(cont, elem)             \
    std::sort(                            \
            (cont).begin(), (cont).end(), \
            [](const decltype(cont)::value_type& a, const decltype(cont)::value_type& b) { return a.elem > b.elem; })
#define sort_asc(cont, elem)              \
    std::sort(                            \
            (cont).begin(), (cont).end(), \
            [](const decltype(cont)::value_type& a, const decltype(cont)::value_type& b) { return a.elem < b.elem; })
#define sort_ptr_desc(cont, elem)         \
    std::sort(                            \
            (cont).begin(), (cont).end(), \
            [](const decltype(cont)::value_type a, const decltype(cont)::value_type b) { return a->elem > b->elem; })
#define sort_ptr_asc(cont, elem)          \
    std::sort(                            \
            (cont).begin(), (cont).end(), \
            [](const decltype(cont)::value_type a, const decltype(cont)::value_type b) { return a->elem < b->elem; })

namespace htwk {

template <class Container>
auto min_element_transformed(Container& c, std::function<float(const typename Container::value_type&)> trans) {
    return std::min_element(c.begin(), c.end(),
                            [&trans](const typename Container::value_type& a, const typename Container::value_type& b) {
                                return trans(a) < trans(b);
                            });
}

template <class Container>
auto sort(Container& c) {
    return std::sort(c.begin(), c.end());
}

template <class Container, class Compare>
auto sort(Container& c, Compare comp) {
    return std::sort(c.begin(), c.end(), comp);
}

template <class Container, class Compare>
auto min_element(Container& c, Compare comp) {
    return std::min_element(c.begin(), c.end(), comp);
}

template <class Container>
auto min_element(Container& c) {
    return std::min_element(c.begin(), c.end());
}

template <class Container, class Compare>
auto max_element(Container& c, Compare comp) {
    return std::max_element(c.begin(), c.end(), comp);
}

template <class Container>
auto max_element(Container& c) {
    return std::max_element(c.begin(), c.end());
}

template <class Container, class OutputIterator>
OutputIterator copy(const Container& in, OutputIterator out) {
    return std::copy(std::begin(in), std::end(in), out);
}

template <typename Func, class Container>
auto for_each(Container& in, const Func& func) {
    return std::for_each(std::begin(in), std::end(in), func);
}

template <class Container>
auto accumulate(Container& in, const typename Container::value_type& init) {
    return std::accumulate(in.begin(), in.end(), init);
}

template <class Container>
auto accumulate(Container& in) {
    return std::accumulate(in.begin(), in.end(), static_cast<typename Container::value_type>(0));
}

template <class Container, typename Accumulator, class BinaryOperation>
auto accumulate(Container& in, const Accumulator& init, BinaryOperation op) {
    return std::accumulate(in.begin(), in.end(), init, op);
}

template <class Container>
auto count(Container& in, const typename Container::value_type& val) {
    return std::count(in.begin(), in.end(), val);
}

template <class Container, class UnaryPredicate>
auto count_if(const Container& in, UnaryPredicate pred) {
    return std::count_if(std::begin(in), std::end(in), pred);
}

template <class Container, class UnaryPredicate>
bool any_of(const Container& in, UnaryPredicate pred) {
    return std::any_of(std::begin(in), std::end(in), pred);
}

template <class Container, class OutputIterator, class UnaryPredicate>
void transform(Container& in, OutputIterator out, UnaryPredicate pred) {
    std::transform(std::begin(in), std::end(in), out, pred);
}

template <class Container, class ContainerOut, class UnaryPredicate>
void transform_insert(Container& in, ContainerOut& out, UnaryPredicate pred) {
    std::transform(std::begin(in), std::end(in), std::back_inserter(out), pred);
}

template <class Container, class ContainerOut, class UnaryPredicate>
void insert_if(Container& in, ContainerOut& out, UnaryPredicate pred) {
    if constexpr (std::is_same<ContainerOut, std::vector<typename ContainerOut::value_type,
                                                         typename ContainerOut::allocator_type>>::value ||
                  std::is_same<ContainerOut, std::deque<typename ContainerOut::value_type,
                                                        typename ContainerOut::allocator_type>>::value ||
                  std::is_same<ContainerOut, std::list<typename ContainerOut::value_type,
                                                       typename ContainerOut::allocator_type>>::value) {
        std::copy_if(std::begin(in), std::end(in), std::back_inserter(out), pred);
    } else {
        std::copy_if(std::begin(in), std::end(in), std::inserter(out, std::end(out)), pred);
    }
}

template <class Container, class UnaryPredicate>
auto partition(Container& c, UnaryPredicate pred) {
    std::map<int, int> foo;
    return std::partition(std::begin(c), std::end(c), pred);
}

template <class Container, class UnaryPredicate>
void erase_if(Container& c, UnaryPredicate pred) {
    if constexpr (std::is_same<Container, std::vector<typename Container::value_type,
                                                      typename Container::allocator_type>>::value ||
                  std::is_same<Container,
                               std::deque<typename Container::value_type, typename Container::allocator_type>>::value ||
                  std::is_same<Container,
                               std::list<typename Container::value_type, typename Container::allocator_type>>::value) {
        c.erase(std::remove_if(std::begin(c), std::end(c), pred), std::end(c));
    } else if constexpr (std::is_same<Container, std::map<typename Container::key_type, typename Container::mapped_type,
                                                          typename Container::key_compare,
                                                          typename Container::allocator_type>>::value) {
        std::vector<typename Container::key_type> to_delete;
        for (const auto& kv : c) {
            if (pred(kv))
                to_delete.push_back(kv.first);
        }
        for (const auto& k : to_delete)
            c.erase(k);
    } else {
        c.erase(std::remove_if(std::begin(c), std::end(c), pred), std::end(c));
    }
}

template <class Container, class UnaryPredicate>
bool all_of(const Container& c, UnaryPredicate pred) {
    return std::all_of(std::begin(c), std::end(c), pred);
}

template <class Container, class UnaryPredicate>
auto find_with_default(const Container& c, UnaryPredicate pred, const typename Container::value_type& def) {
    auto res = std::find_if(std::begin(c), std::end(c), pred);
    return res == std::end(c) ? def : *res;
}

// Equivalent to
// for (size_t i = 0; i < in1.size(); i++)
//     in1[i] += in2[i];
template <class Container1, class Container2>
void pointwise_pluseq(Container1& in1, const Container2& in2) {
    std::transform(std::begin(in1), std::end(in1), std::begin(in2), std::begin(in1),
                   std::plus<typename Container1::value_type>());
}

// Equivalent to
// for (size_t i = 0; i < in1.size(); i++)
//     in1[i] -= in2[i];
template <class Container1, class Container2>
void pointwise_minuseq(Container1& in1, const Container2& in2) {
    std::transform(std::begin(in1), std::end(in1), std::begin(in2), std::begin(in1),
                   std::minus<typename Container1::value_type>());
}

template <class Container>
typename Container::value_type sum(const Container& in) {
    typename Container::value_type tmp{};
    return std::accumulate(std::begin(in), std::end(in), tmp);
}

inline bool closer_to(const point_2d& what, const point_2d& c1, const point_2d& c2) {
    return c1.dist(what) < c2.dist(what);
}

template <class Container, typename Needle>
bool contains(const Container& c, const Needle& n) {
    return c.find(n) != std::end(c);
}

template <class Container, typename Needle>
std::optional<typename Container::value_type> find(const Container& c, const Needle& n) {
    auto it = c.find(n);
    if (it == std::end(c))
        return std::nullopt;
    return *it;
}

}  // namespace htwk

#endif  // ALGORITHM_EXT_H
