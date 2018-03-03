#ifndef ETHERCAT_INTERFACE_H
#define ETHERCAT_INTERFACE_H

class EthercatInterface
{
public:

    /**
     * @brief EthercatInterface: ToDo
     */
    EthercatInterface();

    bool initialize(const std::string& ifname);

    // ToDo: statecheck

    void sendAll();

    void receiveAll();

private:

    bool pdo_transfer_active_;
    volatile int wkc_, expected_wkc_;  // ToDo: correct usage of volatile?
    char IOmap_[4096];
};

#endif // ETHERCAT_INTERFACE_H
