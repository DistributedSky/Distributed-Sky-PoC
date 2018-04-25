import json
from collections import namedtuple

from shapely.geometry import LineString, Point, MultiLineString

from dsky_main.resolvers import GlobalRegionResolver, RegionResolver

ServicedRoute = namedtuple('ServiceRoute', ['route', 'service_provider'])


class Segmenter:

    def __init__(self, asp_registry, global_registry, global_registry_address):
        self.global_registry_address = global_registry_address
        self.global_registry = global_registry
        self.asp_registry = asp_registry

    def _segments(self, route, items):
        segments = []
        for region in items:
            intersection = route.intersection(region.geometry)
            if len(intersection) > 0:
                if type(intersection) is not LineString:
                    raise NotImplementedError("Non LineString segments currently are not supported")
                else:
                    segments.append(ServicedRoute(intersection, region))
        return segments

    def segment_by_global_regions(self, route):
        resolver = self._resolve_global_regions(self.global_registry_address)
        return self._segments(route, resolver.items)

    def _resolve_global_regions(self, address):
        data = self.global_registry.get_file_content(address)
        return GlobalRegionResolver(json.loads(data))

    def _resolve_region(self, address):
        data = self.asp_registry.get_file_content(address)
        return RegionResolver(json.loads(data))

    def segments_by_regions(self, segments):
        output_segments = []

        for serviced_route in segments:
            provider_asp_registry_address = serviced_route.service_provider.asp_registry_address
            asp_resolver = self._resolve_region(provider_asp_registry_address)
            region_segments = self._segments(serviced_route.route, asp_resolver.items)
            segments = segments + region_segments

        return output_segments

    def segments(self, route):
        global_regions = self.segment_by_global_regions(route)
        return self.segments_by_regions(global_regions)
