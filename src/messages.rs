// edit for each board!

cfg_select! {
    feature = "front_left" => {
        pub(crate) use ksu_rs_dbc::messages::An1FlWheelspeed as SpeedFrame;
        pub(crate) use ksu_rs_dbc::messages::An1FlShockpot as ShockFrame;
    }
    feature = "front_right" => {
        pub(crate) use ksu_rs_dbc::messages::An1FrWheelspeed as SpeedFrame;
        pub(crate) use ksu_rs_dbc::messages::An1FrShockpot as ShockFrame;
    }
    feature = "rear_unified" => {
        pub(crate) use ksu_rs_dbc::messages::An1RlWheelspeed as SpeedFrameL;
        pub(crate) use ksu_rs_dbc::messages::An1RrWheelspeed as SpeedFrameR;
        pub(crate) use ksu_rs_dbc::messages::An1RlShockpot as ShockFrameL;
        pub(crate) use ksu_rs_dbc::messages::An1RrShockpot as ShockFrameR;
    }
}
