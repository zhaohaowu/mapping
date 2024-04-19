/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * GUO Zhichong <guozhcihong@sensetime.com>
 */
#pragma once

namespace senseAD {

/*
 * @brief A container for any object
 */
class Any {
 public:
    Any() : holder_(nullptr) {}
    template <typename ValueType>
    explicit Any(const ValueType &value)
        : holder_(new (std::nothrow) Holder<ValueType>(value)) {}
    Any(const Any &other)
        : holder_(other.holder_ != nullptr ? other.holder_->Clone() : nullptr) {
    }
    Any &operator=(const Any &other) {
        if (other.holder_ != nullptr) {
            this->holder_ = other.holder_->Clone();
        } else {
            this->holder_ = nullptr;
        }
        return *this;
    }
    virtual ~Any() {
        if (holder_ != nullptr) delete holder_;
    }
    template <typename ValueType>
    ValueType *GetValue() const {
        if (holder_ == nullptr) return nullptr;
        return &(static_cast<Holder<ValueType> *>(holder_)->value_);
    }

 private:
    class PlaceHolder {
     public:
        virtual ~PlaceHolder() = default;
        virtual PlaceHolder *Clone() = 0;
    };
    template <typename ValueType>
    class Holder : public PlaceHolder {
     public:
        explicit Holder(const ValueType &value) : value_(value) {}
        virtual ~Holder() = default;
        virtual PlaceHolder *Clone() { return new Holder(value_); }
        ValueType value_;
    };
    PlaceHolder *holder_;
};
}  // namespace senseAD
