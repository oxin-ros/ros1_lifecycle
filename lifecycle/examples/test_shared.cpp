#include <iostream>
#include <memory>

class Base : public std::enable_shared_from_this<Base>
{
public:
    Base()
    {
        std::cout << "Base constructor" << std::endl;
    };
    virtual ~Base() = default;

    virtual void on_configure() = 0;
    virtual void on_active() = 0;
};

class Publisher
{
public:
    Publisher(std::shared_ptr<Base> base) : base_(base) {};
    virtual ~Publisher() = default;

    void publish()
    {
        std::cout << "All good!" << std::endl;
    }

private:
    std::shared_ptr<Base> base_{nullptr};
};

class Derived : public Base
{
public:
    Derived() : Base()
    {
        std::cout << "Derived constructor" << std::endl;
    };
    ~Derived() = default;

    void on_configure() override
    {
        publisher_ = std::make_shared<Publisher>(shared_from_this());
    }

    void on_active() override
    {
        publisher_->publish();
    }

private:
    std::shared_ptr<Publisher> publisher_{nullptr};
};

int main()
{
    auto test = std::make_shared<Derived>();
    test->on_configure();
    test->on_active();

    return 0;
}
