NAME_COORD_DICT = {
    "wand": {
    position: {
        x: -0.61,
        y: 2.1, 
        z: 0.0,
    },
    orientation: {
        x: 0.0,
        y: 0.0,
        z: -0.796,
        w: 0.605
    }
}
}

function navToFixedPoint() {
    navName = "wand";
    console.log(`Navigating to ${navName} with coordinates:`, NAME_COORD_DICT[navName]);
    executeNavigateToPose(NAME_COORD_DICT[navName]);
}

const startArucoTagSearch = () => {
    const arucoTagSearch = new ROSLIB.ServiceRequest({});
    startTagSearch.callService(arucoTagSearch, function (result) {
        console.log('Tag search result:', result.success, result.message);
    });
};

const startAlignToTag = () => {
    const alignToTagReq = new ROSLIB.ServiceRequest({});
    alignToTag.callService(alignToTagReq, function (result) {
        console.log('Tag align result:', result.success, result.message);
    });
};