from shapely.geometry import shape


class GeoJSONKeywords:
    FEATURE_COLLECTION = "FeatureCollection"
    TYPE = "type"
    FEATURES = "features"
    FEATURE = "Feature"
    PROPERTIES = "properties"
    GEOMETRY = "geometry"


class ASPZone:

    def __init__(self, id, contract_address, geometry):
        self.geometry = geometry
        self.contract_address = contract_address
        self.id = id


class ParseException(BaseException):
    pass


class GeoJSONResolver:

    def __init__(self, data):
        self.data = data
        self.items = []
        self.parse()

    def _create_item(self, properties, geometry):
        raise NotImplementedError()

    def parse(self):
        if self.data[GeoJSONKeywords.TYPE] == GeoJSONKeywords.FEATURE_COLLECTION:
            for feature in self.data[GeoJSONKeywords.FEATURES]:
                if feature[GeoJSONKeywords.TYPE] == GeoJSONKeywords.FEATURE:
                    if GeoJSONKeywords.PROPERTIES in feature and GeoJSONKeywords.GEOMETRY in feature:
                        geometry = shape(feature[GeoJSONKeywords.GEOMETRY])
                        item = self._create_item(feature[GeoJSONKeywords.PROPERTIES], geometry)
                        self.items.append(item)
                    else:
                        raise ParseException("Properties or geometry are not found in feature")
                else:
                    raise ParseException("Non feature found")


class RegionResolver(GeoJSONResolver):

    def _create_item(self, properties, geometry):
        return ASPZone(properties['id'], properties['contractAddress'], geometry)

    def get_by_address(self, address):
        for region in self.items:
            if region.contract_address == address:
                return region
