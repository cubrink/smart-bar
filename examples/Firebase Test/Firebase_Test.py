from google.cloud import firestore, storage
import os
from pathlib import Path

#auth_json = Path(__file__).parent / 'authentication.json'
auth_json = Path(__file__).parent / 't-cogency-346706-e7bc94fbc128.json'
print(auth_json)
# C:\\Users\\sport\\Desktop\\smart-bar\\smart-bar\\Firebase_Test\\authentication.json
# C:\Users\sport\Desktop\smart-bar\smart-bar\examples\Firebase Test
storage_client = storage.Client.from_service_account_json(auth_json)
#storage_client = storage.Client()

db = firestore.Client(project='t-cogency-346706')

print('Hello, World!')