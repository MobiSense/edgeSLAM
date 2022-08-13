//
// Created by Halcao on 2020/4/17.
//

#ifndef EDGE_SLAM_PROPERTY_H
#define EDGE_SLAM_PROPERTY_H

template <typename T>
class Property {

public:
    typedef T value_type;

    Property(T const& val) : value_(val) {}

    // returns a Signal which is fired when the internal value
    // has been changed. The new value is passed as parameter.
    virtual Signal<T> const& on_change() const {
        return on_change_;
    }

    // sets the Property to a new value.
    // on_change() will be emitted.
    virtual void set(T const& value) {
        if (value != value_) {
            value_ = value;
            on_change_.emit(value_);
        }
    }

    // returns the internal value
    virtual T const& get() const { return value_; }

    // if there are any Properties connected to this Property,
    // they won't be notified of any further changes
    virtual void disconnect_auditors() {
        on_change_.disconnect_all();
    }

    // assigns the value of another Property
    virtual Property<T>& operator=(Property<T> const& rhs) {
        set(rhs.value_);
        return *this;
    }

    // assigns a new value to this Property
    virtual Property<T>& operator=(T const& rhs) {
        set(rhs);
        return *this;
    }

    // returns the value of this Property
    T const& operator()() const {
        return Property<T>::get();
    }

private:
    Signal<T> on_change_;

    T value_;
};


#endif //EDGE_SLAM_PROPERTY_H
