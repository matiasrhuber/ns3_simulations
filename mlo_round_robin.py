import ns3ai

class RoundRobin:
    def __init__(self):
        self.counter = 0

    def get_link(self, sta_id, total_links):
        selected = self.counter % total_links
        self.counter += 1
        print(f"[Python] STA {sta_id} → Link {selected}")
        return selected

rr = RoundRobin()

@ns3ai.expose
def get_link(sta_id, total_links):
    return rr.get_link(sta_id, total_links)
