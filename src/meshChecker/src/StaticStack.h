#ifndef STATICSTACK_H
#define STATICSTACK_H

template<typename T, size_t N=512>
class StaticStack
{
public:
    using value_type = T;

    StaticStack() = default;

    StaticStack(const StaticStack&) = delete;
    StaticStack(StaticStack&&) = delete;

    StaticStack& operator=(const StaticStack&) = delete;
    StaticStack* operator=(StaticStack&&) = delete;

    void Push(const value_type& v) { vals_[size_++] = v; }
    void Push(value_type&& v) { vals_[size_++] = std::move(v); }

    T Pop() { return vals_[--size_]; }

    T& Back() { return vals_[size_-1]; }

    bool Empty() const { return size_ == 0; }

private:
    size_t size_{};
    value_type vals_[N];
};

#endif // STATICSTACK_H
