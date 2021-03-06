// **********************************************************************
//
// Copyright (c) 2003-2014 ZeroC, Inc. All rights reserved.
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
// Generated from file `encoders.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

(function(global, r)
{
    var require = typeof(r) === "function" ? r : function(){};
    require("Ice/Object");
    require("Ice/ObjectPrx");
    require("Ice/Operation");
    require("Ice/Long");
    require("Ice/HashMap");
    require("Ice/HashUtil");
    require("Ice/ArrayUtil");
    require("Ice/StreamHelpers");
    
    var Ice = global.Ice || {};
    require("common");
    
    var jderobot = global.jderobot || {};

    jderobot.EncodersData = Slice.defineObject(
        function(robotx, roboty, robottheta, robotcos, robotsin)
        {
            Ice.Object.call(this);
            this.robotx = robotx !== undefined ? robotx : 0.0;
            this.roboty = roboty !== undefined ? roboty : 0.0;
            this.robottheta = robottheta !== undefined ? robottheta : 0.0;
            this.robotcos = robotcos !== undefined ? robotcos : 0.0;
            this.robotsin = robotsin !== undefined ? robotsin : 0.0;
        },
        Ice.Object, undefined, 1,
        [
            "::Ice::Object",
            "::jderobot::EncodersData"
        ],
        -1,
        function(__os)
        {
            __os.writeFloat(this.robotx);
            __os.writeFloat(this.roboty);
            __os.writeFloat(this.robottheta);
            __os.writeFloat(this.robotcos);
            __os.writeFloat(this.robotsin);
        },
        function(__is)
        {
            this.robotx = __is.readFloat();
            this.roboty = __is.readFloat();
            this.robottheta = __is.readFloat();
            this.robotcos = __is.readFloat();
            this.robotsin = __is.readFloat();
        },
        false);

    jderobot.EncodersDataPrx = Slice.defineProxy(Ice.ObjectPrx, jderobot.EncodersData.ice_staticId, undefined);

    Slice.defineOperations(jderobot.EncodersData, jderobot.EncodersDataPrx);

    /**
     * Interface to the Gazebo encoders sensor.
     **/
    jderobot.Encoders = Slice.defineObject(
        undefined,
        Ice.Object, undefined, 1,
        [
            "::Ice::Object",
            "::jderobot::Encoders"
        ],
        -1, undefined, undefined, false);

    jderobot.EncodersPrx = Slice.defineProxy(Ice.ObjectPrx, jderobot.Encoders.ice_staticId, undefined);

    Slice.defineOperations(jderobot.Encoders, jderobot.EncodersPrx,
    {
        "getEncodersData": [, 2, 2, , , ["jderobot.EncodersData", true], , , , , true]
    });
    global.jderobot = jderobot;
}
(typeof (global) === "undefined" ? window : global, typeof (require) === "undefined" ? undefined : require));
