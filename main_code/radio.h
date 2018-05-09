namespace nustars {
    class Radio {
    private:
        const long TIMEOUT = 1000; //time until we assume the ground station is lost
        long lastContact;
        bool alive;
        bool checkLife();
    public:
        void tick();
        bool isAlive() const;
    };
}
