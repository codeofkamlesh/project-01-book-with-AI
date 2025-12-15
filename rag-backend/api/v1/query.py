from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime

# Import using absolute imports for the package
from rag_backend.pipeline.rag import RAGPipeline
from rag_backend.db.pg_client import get_user_profile

router = APIRouter()

class QueryRequest(BaseModel):
    query: str
    user_id: Optional[str] = None
    context_ids: Optional[List[str]] = []
    highlight_text: Optional[str] = None

class QueryResponse(BaseModel):
    answer: str
    sources: List[Dict[str, Any]]
    query_id: str
    timestamp: datetime

# Initialize RAG pipeline
rag_pipeline = RAGPipeline()

@router.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Main RAG query endpoint that processes user queries and returns contextual answers
    """
    try:
        # Get user profile if user_id is provided
        user_profile = None
        if request.user_id:
            user_profile = await get_user_profile(request.user_id)

        # Process query through RAG pipeline
        result = await rag_pipeline.process_query(
            query=request.query,
            user_profile=user_profile,
            highlight_text=request.highlight_text
        )

        response = QueryResponse(
            answer=result["answer"],
            sources=result["sources"],
            query_id=f"query_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{abs(hash(request.query)) % 10000}",
            timestamp=datetime.now()
        )

        return response

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Query processing failed: {str(e)}")


class SessionContextRequest(BaseModel):
    session_id: str
    query: str
    response: str
    context: Dict[str, Any]

@router.post("/session/answer-context")
async def save_session_context(request: SessionContextRequest):
    """
    Save conversation context to maintain chat history across pages
    """
    # Implementation would save session context to database
    # For now, returning success
    return {"status": "success", "session_id": request.session_id}