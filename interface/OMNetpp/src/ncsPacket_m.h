//
// Generated file, do not edit! Created by nedtool 5.0 from ncsPacket.msg.
//

#ifndef __NCS_NCSPACKET_M_H
#define __NCS_NCSPACKET_M_H

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0500
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif


namespace ncs {

// cplusplus {{
#include <vector>
typedef std::vector<double> doubleVector;
// }}

/**
 * Class generated from <tt>ncsPacket.msg:27</tt> by nedtool.
 * <pre>
 * //
 * // TODO generated message class
 * //
 * packet NcsPacket
 * {
 *     //string value;
 *     double channelDelay;
 *     bool isDropped;
 *     doubleVector values;
 * }
 * </pre>
 */
class NcsPacket : public ::omnetpp::cPacket
{
  protected:
    double channelDelay;
    bool isDropped;
    doubleVector values;

  private:
    void copy(const NcsPacket& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const NcsPacket&);

  public:
    NcsPacket(const char *name=nullptr, int kind=0);
    NcsPacket(const NcsPacket& other);
    virtual ~NcsPacket();
    NcsPacket& operator=(const NcsPacket& other);
    virtual NcsPacket *dup() const {return new NcsPacket(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b);

    // field getter/setter methods
    virtual double getChannelDelay() const;
    virtual void setChannelDelay(double channelDelay);
    virtual bool getIsDropped() const;
    virtual void setIsDropped(bool isDropped);
    virtual doubleVector& getValues();
    virtual const doubleVector& getValues() const {return const_cast<NcsPacket*>(this)->getValues();}
    virtual void setValues(const doubleVector& values);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const NcsPacket& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, NcsPacket& obj) {obj.parsimUnpack(b);}

} // namespace ncs

#endif // ifndef __NCS_NCSPACKET_M_H

