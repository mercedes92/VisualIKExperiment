// **********************************************************************
//
// Copyright (c) 2003-2013 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.5.1
//
// <auto-generated>
//
// Generated from file `Reflexxes.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef ____Reflexxes_h__
#define ____Reflexxes_h__

#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/Outgoing.h>
#include <Ice/OutgoingAsync.h>
#include <Ice/Incoming.h>
#include <Ice/Direct.h>
#include <IceUtil/ScopedArray.h>
#include <IceUtil/Optional.h>
#include <Ice/StreamF.h>
#include <Ice/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 305
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 1
#       error Ice patch level mismatch!
#   endif
#endif

namespace IceProxy
{

namespace RoboCompReflexxes
{

class Reflexxes;
void __read(::IceInternal::BasicStream*, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompReflexxes::Reflexxes>&);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompReflexxes::Reflexxes*);

}

}

namespace RoboCompReflexxes
{

class Reflexxes;
bool operator==(const Reflexxes&, const Reflexxes&);
bool operator<(const Reflexxes&, const Reflexxes&);
::Ice::Object* upCast(::RoboCompReflexxes::Reflexxes*);
typedef ::IceInternal::Handle< ::RoboCompReflexxes::Reflexxes> ReflexxesPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompReflexxes::Reflexxes> ReflexxesPrx;
void __patch(ReflexxesPtr&, const ::Ice::ObjectPtr&);

}

namespace RoboCompReflexxes
{

struct Motor
{
    ::std::string name;
    ::Ice::Float angle;
    ::Ice::Float speed;

    bool operator==(const Motor& __rhs) const
    {
        if(this == &__rhs)
        {
            return true;
        }
        if(name != __rhs.name)
        {
            return false;
        }
        if(angle != __rhs.angle)
        {
            return false;
        }
        if(speed != __rhs.speed)
        {
            return false;
        }
        return true;
    }

    bool operator<(const Motor& __rhs) const
    {
        if(this == &__rhs)
        {
            return false;
        }
        if(name < __rhs.name)
        {
            return true;
        }
        else if(__rhs.name < name)
        {
            return false;
        }
        if(angle < __rhs.angle)
        {
            return true;
        }
        else if(__rhs.angle < angle)
        {
            return false;
        }
        if(speed < __rhs.speed)
        {
            return true;
        }
        else if(__rhs.speed < speed)
        {
            return false;
        }
        return false;
    }

    bool operator!=(const Motor& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Motor& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Motor& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Motor& __rhs) const
    {
        return !operator<(__rhs);
    }
};

typedef ::std::vector< ::RoboCompReflexxes::Motor> MotorAngleList;

}

namespace Ice
{
template<>
struct StreamableTraits< ::RoboCompReflexxes::Motor>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 9;
    static const bool fixedLength = false;
};

template<class S>
struct StreamWriter< ::RoboCompReflexxes::Motor, S>
{
    static void write(S* __os, const ::RoboCompReflexxes::Motor& v)
    {
        __os->write(v.name);
        __os->write(v.angle);
        __os->write(v.speed);
    }
};

template<class S>
struct StreamReader< ::RoboCompReflexxes::Motor, S>
{
    static void read(S* __is, ::RoboCompReflexxes::Motor& v)
    {
        __is->read(v.name);
        __is->read(v.angle);
        __is->read(v.speed);
    }
};

}

namespace RoboCompReflexxes
{

class Callback_Reflexxes_setJointPosition_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_Reflexxes_setJointPosition_Base> Callback_Reflexxes_setJointPositionPtr;

class Callback_Reflexxes_getStatePosition_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_Reflexxes_getStatePosition_Base> Callback_Reflexxes_getStatePositionPtr;

}

namespace IceProxy
{

namespace RoboCompReflexxes
{

class Reflexxes : virtual public ::IceProxy::Ice::Object
{
public:

    void setJointPosition(const ::RoboCompReflexxes::MotorAngleList& newAnglesOfMotors)
    {
        setJointPosition(newAnglesOfMotors, 0);
    }
    void setJointPosition(const ::RoboCompReflexxes::MotorAngleList& newAnglesOfMotors, const ::Ice::Context& __ctx)
    {
        setJointPosition(newAnglesOfMotors, &__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_setJointPosition(const ::RoboCompReflexxes::MotorAngleList& newAnglesOfMotors, const ::IceInternal::Function<void ()>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return begin_setJointPosition(newAnglesOfMotors, 0, new ::IceInternal::Cpp11FnOnewayCallbackNC(__response, __exception, __sent));
    }
    ::Ice::AsyncResultPtr
    begin_setJointPosition(const ::RoboCompReflexxes::MotorAngleList& newAnglesOfMotors, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_setJointPosition(newAnglesOfMotors, 0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_setJointPosition(const ::RoboCompReflexxes::MotorAngleList& newAnglesOfMotors, const ::Ice::Context& __ctx, const ::IceInternal::Function<void ()>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return begin_setJointPosition(newAnglesOfMotors, &__ctx, new ::IceInternal::Cpp11FnOnewayCallbackNC(__response, __exception, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_setJointPosition(const ::RoboCompReflexxes::MotorAngleList& newAnglesOfMotors, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_setJointPosition(newAnglesOfMotors, &__ctx, ::Ice::newCallback(__completed, __sent));
    }
#endif

    ::Ice::AsyncResultPtr begin_setJointPosition(const ::RoboCompReflexxes::MotorAngleList& newAnglesOfMotors)
    {
        return begin_setJointPosition(newAnglesOfMotors, 0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_setJointPosition(const ::RoboCompReflexxes::MotorAngleList& newAnglesOfMotors, const ::Ice::Context& __ctx)
    {
        return begin_setJointPosition(newAnglesOfMotors, &__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_setJointPosition(const ::RoboCompReflexxes::MotorAngleList& newAnglesOfMotors, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_setJointPosition(newAnglesOfMotors, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_setJointPosition(const ::RoboCompReflexxes::MotorAngleList& newAnglesOfMotors, const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_setJointPosition(newAnglesOfMotors, &__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_setJointPosition(const ::RoboCompReflexxes::MotorAngleList& newAnglesOfMotors, const ::RoboCompReflexxes::Callback_Reflexxes_setJointPositionPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_setJointPosition(newAnglesOfMotors, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_setJointPosition(const ::RoboCompReflexxes::MotorAngleList& newAnglesOfMotors, const ::Ice::Context& __ctx, const ::RoboCompReflexxes::Callback_Reflexxes_setJointPositionPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_setJointPosition(newAnglesOfMotors, &__ctx, __del, __cookie);
    }

    void end_setJointPosition(const ::Ice::AsyncResultPtr&);
    
private:

    void setJointPosition(const ::RoboCompReflexxes::MotorAngleList&, const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_setJointPosition(const ::RoboCompReflexxes::MotorAngleList&, const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:

    bool getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors)
    {
        return getStatePosition(anglesOfMotors, 0);
    }
    bool getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors, const ::Ice::Context& __ctx)
    {
        return getStatePosition(anglesOfMotors, &__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors, const ::IceInternal::Function<void (bool)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getStatePosition(anglesOfMotors, 0, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getStatePosition(anglesOfMotors, 0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (bool)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getStatePosition(anglesOfMotors, &__ctx, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getStatePosition(anglesOfMotors, &__ctx, ::Ice::newCallback(__completed, __sent));
    }
    
private:

    ::Ice::AsyncResultPtr __begin_getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors, const ::Ice::Context* __ctx, const ::IceInternal::Function<void (bool)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception, const ::IceInternal::Function<void (bool)>& __sent)
    {
        class Cpp11CB : public ::IceInternal::Cpp11FnCallbackNC
        {
        public:

            Cpp11CB(const ::std::function<void (bool)>& responseFunc, const ::std::function<void (const ::Ice::Exception&)>& exceptionFunc, const ::std::function<void (bool)>& sentFunc) :
                ::IceInternal::Cpp11FnCallbackNC(exceptionFunc, sentFunc),
                _response(responseFunc)
            {
                CallbackBase::checkCallback(true, responseFunc || exceptionFunc != nullptr);
            }

            virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
            {
                ::RoboCompReflexxes::ReflexxesPrx __proxy = ::RoboCompReflexxes::ReflexxesPrx::uncheckedCast(__result->getProxy());
                bool __ret;
                try
                {
                    __ret = __proxy->end_getStatePosition(__result);
                }
                catch(::Ice::Exception& ex)
                {
                    Cpp11FnCallbackNC::__exception(__result, ex);
                    return;
                }
                if(_response != nullptr)
                {
                    _response(__ret);
                }
            }
        
        private:
            
            ::std::function<void (bool)> _response;
        };
        return begin_getStatePosition(anglesOfMotors, __ctx, new Cpp11CB(__response, __exception, __sent));
    }
    
public:
#endif

    ::Ice::AsyncResultPtr begin_getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors)
    {
        return begin_getStatePosition(anglesOfMotors, 0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors, const ::Ice::Context& __ctx)
    {
        return begin_getStatePosition(anglesOfMotors, &__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getStatePosition(anglesOfMotors, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors, const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getStatePosition(anglesOfMotors, &__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors, const ::RoboCompReflexxes::Callback_Reflexxes_getStatePositionPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getStatePosition(anglesOfMotors, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getStatePosition(const ::RoboCompReflexxes::MotorAngleList& anglesOfMotors, const ::Ice::Context& __ctx, const ::RoboCompReflexxes::Callback_Reflexxes_getStatePositionPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getStatePosition(anglesOfMotors, &__ctx, __del, __cookie);
    }

    bool end_getStatePosition(const ::Ice::AsyncResultPtr&);
    
private:

    bool getStatePosition(const ::RoboCompReflexxes::MotorAngleList&, const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_getStatePosition(const ::RoboCompReflexxes::MotorAngleList&, const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_adapterId(const ::std::string& __id) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_locatorCacheTimeout(int __timeout) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_connectionCached(bool __cached) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_secure(bool __secure) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_preferSecure(bool __preferSecure) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_collocationOptimized(bool __co) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_twoway() const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_twoway().get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_oneway() const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_oneway().get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_batchOneway() const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_datagram() const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_datagram().get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_batchDatagram() const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_compress(bool __compress) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_timeout(int __timeout) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_connectionId(const ::std::string& __id) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<Reflexxes> ice_encodingVersion(const ::Ice::EncodingVersion& __v) const
    {
        return dynamic_cast<Reflexxes*>(::IceProxy::Ice::Object::ice_encodingVersion(__v).get());
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

namespace IceDelegate
{

namespace RoboCompReflexxes
{

class Reflexxes : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void setJointPosition(const ::RoboCompReflexxes::MotorAngleList&, const ::Ice::Context*, ::IceInternal::InvocationObserver&) = 0;

    virtual bool getStatePosition(const ::RoboCompReflexxes::MotorAngleList&, const ::Ice::Context*, ::IceInternal::InvocationObserver&) = 0;
};

}

}

namespace IceDelegateM
{

namespace RoboCompReflexxes
{

class Reflexxes : virtual public ::IceDelegate::RoboCompReflexxes::Reflexxes,
                  virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void setJointPosition(const ::RoboCompReflexxes::MotorAngleList&, const ::Ice::Context*, ::IceInternal::InvocationObserver&);

    virtual bool getStatePosition(const ::RoboCompReflexxes::MotorAngleList&, const ::Ice::Context*, ::IceInternal::InvocationObserver&);
};

}

}

namespace IceDelegateD
{

namespace RoboCompReflexxes
{

class Reflexxes : virtual public ::IceDelegate::RoboCompReflexxes::Reflexxes,
                  virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void setJointPosition(const ::RoboCompReflexxes::MotorAngleList&, const ::Ice::Context*, ::IceInternal::InvocationObserver&);

    virtual bool getStatePosition(const ::RoboCompReflexxes::MotorAngleList&, const ::Ice::Context*, ::IceInternal::InvocationObserver&);
};

}

}

namespace RoboCompReflexxes
{

class Reflexxes : virtual public ::Ice::Object
{
public:

    typedef ReflexxesPrx ProxyType;
    typedef ReflexxesPtr PointerType;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void setJointPosition(const ::RoboCompReflexxes::MotorAngleList&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setJointPosition(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual bool getStatePosition(const ::RoboCompReflexxes::MotorAngleList&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getStatePosition(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

protected:
    virtual void __writeImpl(::IceInternal::BasicStream*) const;
    virtual void __readImpl(::IceInternal::BasicStream*);
    #ifdef __SUNPRO_CC
    using ::Ice::Object::__writeImpl;
    using ::Ice::Object::__readImpl;
    #endif
};

inline bool operator==(const Reflexxes& l, const Reflexxes& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

inline bool operator<(const Reflexxes& l, const Reflexxes& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

}

namespace RoboCompReflexxes
{

template<class T>
class CallbackNC_Reflexxes_setJointPosition : public Callback_Reflexxes_setJointPosition_Base, public ::IceInternal::OnewayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)();

    CallbackNC_Reflexxes_setJointPosition(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallbackNC<T>(obj, cb, excb, sentcb)
    {
    }
};

template<class T> Callback_Reflexxes_setJointPositionPtr
newCallback_Reflexxes_setJointPosition(const IceUtil::Handle<T>& instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Reflexxes_setJointPosition<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Reflexxes_setJointPositionPtr
newCallback_Reflexxes_setJointPosition(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Reflexxes_setJointPosition<T>(instance, 0, excb, sentcb);
}

template<class T> Callback_Reflexxes_setJointPositionPtr
newCallback_Reflexxes_setJointPosition(T* instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Reflexxes_setJointPosition<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Reflexxes_setJointPositionPtr
newCallback_Reflexxes_setJointPosition(T* instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Reflexxes_setJointPosition<T>(instance, 0, excb, sentcb);
}

template<class T, typename CT>
class Callback_Reflexxes_setJointPosition : public Callback_Reflexxes_setJointPosition_Base, public ::IceInternal::OnewayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const CT&);

    Callback_Reflexxes_setJointPosition(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallback<T, CT>(obj, cb, excb, sentcb)
    {
    }
};

template<class T, typename CT> Callback_Reflexxes_setJointPositionPtr
newCallback_Reflexxes_setJointPosition(const IceUtil::Handle<T>& instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Reflexxes_setJointPosition<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Reflexxes_setJointPositionPtr
newCallback_Reflexxes_setJointPosition(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Reflexxes_setJointPosition<T, CT>(instance, 0, excb, sentcb);
}

template<class T, typename CT> Callback_Reflexxes_setJointPositionPtr
newCallback_Reflexxes_setJointPosition(T* instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Reflexxes_setJointPosition<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Reflexxes_setJointPositionPtr
newCallback_Reflexxes_setJointPosition(T* instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Reflexxes_setJointPosition<T, CT>(instance, 0, excb, sentcb);
}

template<class T>
class CallbackNC_Reflexxes_getStatePosition : public Callback_Reflexxes_getStatePosition_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(bool);

    CallbackNC_Reflexxes_getStatePosition(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), response(cb)
    {
    }

    virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompReflexxes::ReflexxesPrx __proxy = ::RoboCompReflexxes::ReflexxesPrx::uncheckedCast(__result->getProxy());
        bool __ret;
        try
        {
            __ret = __proxy->end_getStatePosition(__result);
        }
        catch(::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::__exception(__result, ex);
            return;
        }
        if(response)
        {
            (::IceInternal::CallbackNC<T>::callback.get()->*response)(__ret);
        }
    }

    Response response;
};

template<class T> Callback_Reflexxes_getStatePositionPtr
newCallback_Reflexxes_getStatePosition(const IceUtil::Handle<T>& instance, void (T::*cb)(bool), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Reflexxes_getStatePosition<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Reflexxes_getStatePositionPtr
newCallback_Reflexxes_getStatePosition(T* instance, void (T::*cb)(bool), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Reflexxes_getStatePosition<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_Reflexxes_getStatePosition : public Callback_Reflexxes_getStatePosition_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(bool, const CT&);

    Callback_Reflexxes_getStatePosition(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), response(cb)
    {
    }

    virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompReflexxes::ReflexxesPrx __proxy = ::RoboCompReflexxes::ReflexxesPrx::uncheckedCast(__result->getProxy());
        bool __ret;
        try
        {
            __ret = __proxy->end_getStatePosition(__result);
        }
        catch(::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::__exception(__result, ex);
            return;
        }
        if(response)
        {
            (::IceInternal::Callback<T, CT>::callback.get()->*response)(__ret, CT::dynamicCast(__result->getCookie()));
        }
    }

    Response response;
};

template<class T, typename CT> Callback_Reflexxes_getStatePositionPtr
newCallback_Reflexxes_getStatePosition(const IceUtil::Handle<T>& instance, void (T::*cb)(bool, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Reflexxes_getStatePosition<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Reflexxes_getStatePositionPtr
newCallback_Reflexxes_getStatePosition(T* instance, void (T::*cb)(bool, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Reflexxes_getStatePosition<T, CT>(instance, cb, excb, sentcb);
}

}

#endif