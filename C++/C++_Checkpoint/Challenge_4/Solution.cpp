/*Here's my solution:

class Car {
    private:
        bool in_working_condition_;

    public:
        Car();
        void wearAndTear();
        bool drive();
        void fix();
};

The Car class is pretty straightforward. The trickiest part, I found, was making sure that the constructor was defined.

Note, the trailing _ on in_working_condition_ is common tactic for designating private properties in C++.
*/