import os
from tempfile import TemporaryDirectory

from base58 import b58encode, b58decode


class IPFSConnector:

    def __init__(self, ipfsapi):
        self.ipfsapi = ipfsapi

    def add_to_ipfs(self, content):
        with TemporaryDirectory() as tmpdir:
            workdir = os.getcwd()
            try:
                os.chdir(tmpdir)
                with open("file_to_upload", "w") as ipfs_file:
                    ipfs_file.write(content)

                ipfs_file_hash = self.ipfsapi.add("file_to_upload")['Hash']
                return ipfs_file_hash
            finally:
                os.chdir(workdir)

    def get_from_ipfs(self, ipfs_file_hash):
        with TemporaryDirectory() as tmpdir:
            workdir = os.getcwd()
            try:
                os.chdir(tmpdir)
                self.ipfsapi.get(ipfs_file_hash)
                with open(ipfs_file_hash, "r") as ipfs_file:
                    result = ipfs_file.read()
                    return result
            finally:
                print(workdir)
                os.chdir(workdir)


class IPFSFileRegistry:

    def __init__(self, abi, web3, ipfsapi, eth_account=None):

        self.eth_account = eth_account
        self.web3 = web3
        self.ipfs_connector = IPFSConnector(ipfsapi)
        self.abi = abi

    def _contract(self, address):
        return self.web3.eth.contract(address, abi=self.abi)

    def get_file_content(self, address):
        contract = self._contract(address)
        ipfs_file_hash_part = contract.functions.fileHash().call()
        ipfs_file_hash = b58encode(bytes([0x12, 0x20]) + ipfs_file_hash_part)
        return self.get_from_ipfs(ipfs_file_hash)

    def post_ipfs_hash(self, address, ipfs_file_hash, force=False):
        if self.eth_account is None:
            raise ValueError("Eth account is not provided")
        contract = self._contract(address)
        new_hash_bytes = b58decode(ipfs_file_hash)[2:]
        current_hash_bytes = contract.functions.fileHash().call()
        if current_hash_bytes != new_hash_bytes or force:
            contract.functions.changeFile(new_hash_bytes).transact({'from': self.eth_account})

    def post_file_content(self, address, data):
        ipfs_file_hash = self.add_to_ipfs(data)
        self.post_ipfs_hash(address, ipfs_file_hash)
        return ipfs_file_hash

    def add_to_ipfs(self, content):
        return self.ipfs_connector.add_to_ipfs(content)

    def get_from_ipfs(self, ipfs_file_hash):
        return self.ipfs_connector.get_from_ipfs(ipfs_file_hash)