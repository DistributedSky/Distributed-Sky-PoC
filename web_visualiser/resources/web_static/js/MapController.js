function MapController(mapDivId, initialView) {

    let map;
    let routes = {};
    let styles = {
        'default': {color: 'red', weight: 15, opacity: 0.5}
    };

    function initMap() {
        map = L.map(mapDivId).setView(initialView['position'], initialView['scale']);
        const osmUrl = 'http://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';
        const osmAttrib = 'Map data © <a href="http://openstreetmap.org">OpenStreetMap</a> contributors';
        const osm = new L.TileLayer(osmUrl, {minZoom: 1, maxZoom: 12, attribution: osmAttrib});
        map.addLayer(osm);

    }

    function addObject(objectId, geoJSONDescription, styleId) {
        let polyline = L.geoJSON(geoJSONDescription, {
            style: styles[styleId]
        }).addTo(map);
        routes[objectId] = polyline;
        return polyline;
    }

    function setStyleTo(objectId, styleId) {
        routes[objectId].setStyle(styles[styleId]);
    }

    function setStyle(styleId, styleParams) {
        styles[styleId] = styleParams;
    }

    let that = {
        init: initMap,
        addRoute: addObject,
        setStyleTo: setStyleTo,
        setStyle: setStyle
    };

    return that;
}