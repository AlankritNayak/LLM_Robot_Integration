from pydantic import BaseModel
from typing import List, Optional

class UserMessage(BaseModel):
    message: str